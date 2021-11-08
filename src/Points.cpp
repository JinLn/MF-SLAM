#include"Points.h"
#include"Ncc.h"
using namespace std;
namespace ORB_SLAM3 {

    int Kfs;
    int squenceimg;
    Point LeftUp, RightBottom;
    float Fscale;
    float invFscale;
    int height;
    int width;
    float fx;
    float Rfx;
    float Lcx;
    float Rcx;
    float Rcy;
    float CamY;
    double  duration;
    double  allduration = 0;
    double  averduration = 0;
    int summatchimg = 0;
    float myscaleFactor;
    int matchednums;
    cv::Mat LEFTD, LEFTK, LEFTR, LEFTP;
    cv::Mat RIGHTD, RIGHTK, RIGHTR, RIGHTP;

    void Calibfind(const string &pathset) {

        // The first ways to find ROI area in left img and acquire  LeftUp and  RightBottom points
        cout << "-------------------- Calib Ways ----------------------------- " << endl ;
        cv::FileStorage fSettings( pathset, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float Rfx = fSettings["rightCamera.fx"];
        ORB_SLAM3::height = fSettings["Camera.height"];
        ORB_SLAM3::width  = fSettings["Camera.width"];

        // Rcx must big than Lcx
        ORB_SLAM3::Lcx = fSettings["Camera.cx"];
        ORB_SLAM3::Rcx = fSettings["rightCamera.cx"];
        ORB_SLAM3::Rcy = fSettings["rightCamera.cy"];
        ORB_SLAM3::CamY = fSettings["Camera.y"];

        ORB_SLAM3::Fscale = Rfx / fx;
        ORB_SLAM3::invFscale = 1 / ORB_SLAM3::Fscale;
        // Lcx Or Rcx ???
        ORB_SLAM3::LeftUp.x = max(ORB_SLAM3::Rcx * (1 - ORB_SLAM3::invFscale), 0.0f);
        ORB_SLAM3::LeftUp.y = max(ORB_SLAM3::Rcy * (1 - ORB_SLAM3::invFscale), 0.0f );
        //ORB_SLAM3::LeftUp.x = 160; ORB_SLAM3::LeftUp.y = 107;
        ORB_SLAM3::RightBottom.x = min( ORB_SLAM3::LeftUp.x + ORB_SLAM3::invFscale * ORB_SLAM3::width, (float)ORB_SLAM3::width - 1) ;
        ORB_SLAM3::RightBottom.y = min( ORB_SLAM3::LeftUp.y + ORB_SLAM3::invFscale * ORB_SLAM3::height, (float)ORB_SLAM3::height - 1);
        cout << "LeftUp point = " << ORB_SLAM3::LeftUp << "  |  " << " RightBottom = " << ORB_SLAM3::RightBottom << endl;
        return ;
    }

    void Nccfind(const string &pathset, vector<string> &vstrImageLeft, vector<string> &vstrImageRight) {
        // zhe other ways by jinln
        // Use NCC to find ROI area in left img and acquire  LeftUp and  RightBottom points;
        // strSettingPath,       //配置文件路径
        cout << "-------------------- Ncc Ways ----------------------------- " << endl ;
        cv::FileStorage fSettings( pathset, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float Rfx = fSettings["rightCamera.fx"];
        ORB_SLAM3::height = fSettings["Camera.height"];
        ORB_SLAM3::width  = fSettings["Camera.width"];

        // Rcx must big than Lcx
        ORB_SLAM3::Lcx = fSettings["Camera.cx"];
        ORB_SLAM3::Rcx = fSettings["rightCamera.cx"];

        ORB_SLAM3::Fscale = Rfx / fx;
        ORB_SLAM3::invFscale = 1 / ORB_SLAM3::Fscale;
        Mat L0, R0, imgL0, imgR0;
        L0 = imread(vstrImageLeft[0], cv::IMREAD_UNCHANGED);
        R0 = imread(vstrImageRight[0], cv::IMREAD_UNCHANGED);

        L0.copyTo(imgL0);
        R0.copyTo(imgR0);
        // cout << "Fscale= " << ORB_SLAM3::Fscale << endl;
        ORB_SLAM3::FindPoints( imgL0, imgR0, ORB_SLAM3:: invFscale);

        cout << "LeftUp point = " << ORB_SLAM3::LeftUp << "  |  " << " RightBottom = " << ORB_SLAM3::RightBottom << endl;
        return ;
    }

    void readcameraMat(const string &pathset) {
        cv::FileStorage fSettings( pathset, cv::FileStorage::READ);
        ORB_SLAM3::fx = fSettings["Camera.fx"];
        ORB_SLAM3::Rfx = fSettings["rightCamera.fx"];
        ORB_SLAM3::Fscale = Rfx / fx;
        ORB_SLAM3::invFscale = 1 / ORB_SLAM3::Fscale;
        ORB_SLAM3::Lcx = fSettings["Camera.cx"];
        ORB_SLAM3::Rcx = fSettings["rightCamera.cx"];
        ORB_SLAM3::myscaleFactor = fSettings["ORBextractor.scaleFactor"];
        fSettings["LEFT.D"] >> ORB_SLAM3::LEFTD;
        fSettings["LEFT.K"] >> ORB_SLAM3::LEFTK;
        fSettings["LEFT.R"] >> ORB_SLAM3::LEFTR;
        fSettings["LEFT.P"] >> ORB_SLAM3::LEFTP;
        fSettings["RIGHT.D"] >> ORB_SLAM3::RIGHTD;
        fSettings["RIGHT.K"] >> ORB_SLAM3::RIGHTK;
        fSettings["RIGHT.R"] >> ORB_SLAM3::RIGHTR;
        fSettings["RIGHT.P"] >> ORB_SLAM3::RIGHTP;
        ORB_SLAM3::height = fSettings["Camera.height"];
        ORB_SLAM3::width  = fSettings["Camera.width"];
        if(LEFTD.empty() || LEFTK.empty() || LEFTR.empty() || LEFTP.empty() || RIGHTD.empty() || RIGHTK.empty() || RIGHTR.empty() || RIGHTP.empty() ||
                height == 0 || width == 0 ) {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            exit(-1);
        }
    }
}
