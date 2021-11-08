#ifndef POINTS_H
#define POINTS_H
#include<iostream>
#include<opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include<string>
using namespace cv;
using namespace std;
namespace ORB_SLAM3 {

    //kfs
    extern int Kfs;
    extern cv::Mat LEFTD, LEFTK, LEFTR, LEFTP;
    extern cv::Mat RIGHTD, RIGHTK, RIGHTR, RIGHTP;

    extern int squenceimg;
    extern Point LeftUp, RightBottom;
    extern float Fscale;
    extern float invFscale;
    extern int height;
    extern int width;
    extern float fx;
    extern float Rfx;
    extern float Lcx;
    extern float Rcx;
    extern float Rcy;
    extern float CamY;
    extern double  duration;
    extern double  averduration;
    extern double  allduration;
    extern int summatchimg;
    extern int matchednums; //匹配的数
    extern float myscaleFactor;
    void Calibfind(const string &pathset);
    void Nccfind(const string &pathset, vector<string> &vstrImageLeft, vector<string> &vstrImageRight);

    void readcameraMat(const string &pathset);
}
#endif


