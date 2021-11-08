/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include<Points.h>
#include<opencv2/core/core.hpp>
#include"Ncc.h"
#include<System.h>
#include <cmath>
#include<ctime>
#include<fstream>
#include<iomanip>

using namespace std;
using namespace cv;

void LoadImages(const string &pathtosettings, const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv) {
    if(argc != 4) {
        cerr << endl << "Usage: ./stereo_test path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages( string(argv[2]), string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);

    const int nImages = vstrImageLeft.size();
    // 标定方式找ＲＯＩ
    //ORB_SLAM3::Calibfind( argv[2]);
    // Ncc找ＲＯＩ
    ORB_SLAM3::Nccfind(argv[2], vstrImageLeft, vstrImageRight);
    ORB_SLAM3::readcameraMat(argv[2]);
    //  记录匹配数
    std::string sstr = "./Matchednums.txt";
    std::ofstream fout(sstr);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imLeft, imRight;
    for(int ni = 0; ni < nImages; ni++) {
        ORB_SLAM3::squenceimg = ni;
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni], cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty()) {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft, imRight, tframe);
        // 写入匹配数的数据
        fout << ORB_SLAM3::matchednums << endl;
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if(ni < nImages - 1) {
            T = vTimestamps[ni + 1] - tframe;
        } else if(ni > 0) {
            T = tframe - vTimestamps[ni - 1];
        }

        if(ttrack < T) {
            usleep((T - ttrack) * 1e6);
        }

    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for(int ni = 0; ni < nImages; ni++) {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "KFS: " << ORB_SLAM3::Kfs << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}

void LoadImages(const string &pathtosettings, const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps) {
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof()) {
        string s;
        getline(fTimes, s);
        if(!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    cv::FileStorage fSettings( pathtosettings, cv::FileStorage::READ);
    string strPrefixLeft, strPrefixRight;
    string cameratype = fSettings["Camera.combine"];
    if(cameratype == "01") {
        strPrefixLeft = strPathToSequence + "/image_0/";
        strPrefixRight = strPathToSequence + "/image_1/";
        cerr << "left And Center Camera combine..." << endl;
    } else if(cameratype == "02") {
        strPrefixLeft = strPathToSequence + "/image_0/";
        strPrefixRight = strPathToSequence + "/image_2/";
        cerr << "left And Right Camera combine..." << endl;
    } else if(cameratype == "12") {
        strPrefixLeft = strPathToSequence + "/image_1/";
        strPrefixRight = strPathToSequence + "/image_2/";
        cerr << "Center And Right Camera combine..." << endl;
    } else {
        strPrefixLeft = strPathToSequence + "/image_0/";
        strPrefixRight = strPathToSequence + "/image_1/";
        cerr << "Original Camera combine..." << endl;
        // cerr << "Camera type is error ..."<< endl;
        // exit(-1);
    }

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i = 0; i < nTimes; i++) {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
