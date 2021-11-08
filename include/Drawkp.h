#ifndef DRAWKP_H
#define DRAWKP_H
#include <iostream>
#include <vector>
#include <string>
#include "include/ORBextractor.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


//画出All特征点
void DrawKP(cv::Mat& dstImg, std::vector<cv::KeyPoint>& vKpts);

//打印matched特征点
void DrawKP(cv::Mat& dstImg, std::vector<cv::KeyPoint>& vKpts ,std::vector<float>& mvuflag );

// 绘制匹配点
// imgResMatch 匹配结果图片, vKpRes1第一张点, vKpRes2　第二张点, vMatchedKptsIdin2匹配关系,
void DrawMatch(cv::Mat& imgRes, std::vector<KeyPoint>& vKpRes1, std::vector<KeyPoint>& vKpRes2, std::vector<float>& vMatchedKptsIdin1 ,std::vector<float> vMatchedKptsIdin2);
//合并显示特征点图片
void ShowKeypoints(Mat& img1, Mat& img2, string& S);
//合并返回图片
void ShowKeypoints(Mat& img1, Mat& img2, Mat& img3);
//打印矩阵
void printMat(Mat& matprint);

void myremapimg(Mat& img ,Mat& K ,Mat& D,Mat& R, Mat& P);
#endif
