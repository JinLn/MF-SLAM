#include <iostream>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include<vector>
#include"Ncc.h"
#include<vector>
#include<algorithm>
#include <ctime>
#include"Points.h"
#include "Frame.h"
using namespace std;
using namespace cv;

#include <opencv2/opencv.hpp>

using namespace cv;

namespace ORB_SLAM3
{

void ncclong(Mat& srcImage, Mat& templImage, Mat& result)
{
    int rows = srcImage.rows - templImage.rows + 1;
    assert(templImage.rows <= srcImage.rows);
    int cols = srcImage.cols - templImage.cols + 1;
    assert(templImage.cols <= srcImage.cols);
    result.create(rows, cols, CV_32FC1);

    Mat src, templ;
    if(srcImage.channels()==3 && templImage .channels()== 3)
    {
    cvtColor(srcImage, src, COLOR_BGR2GRAY);
    cvtColor(templImage, templ, COLOR_BGR2GRAY);
    }
    else{
        srcImage.copyTo(src);
        templImage.copyTo(templ);
    }
    double sum_templ = 0.;
    for (int i = 0; i < templ.rows; i++)
    {
        uchar* templ_ptr = templ.ptr<uchar>(i);
        for (int j = 0; j < templ.cols; j++)
        {
            sum_templ += (double)(templ_ptr[j] * templ_ptr[j]);
        }
    }

    double num = 0., den = 0.;
    for (int i = 0; i < result.rows; i++)
    {
        float* result_ptr = result.ptr<float>(i);                       //因为result的类型是CV_32FC1，所以其对应的指针所指向的类型为float，用uchar和double都会出错
        for (int j = 0; j < result.cols; j++)
        {

            for (int m = 0; m < templ.rows; m++)
            {
                uchar* templ_ptr = templ.ptr<uchar>(m);
                uchar* src_ptr = src.ptr<uchar>(i+m);
                for (int n = 0; n < templ.cols; n++)
                {
                    num += (double)(templ_ptr[n] * src_ptr[j + n]);
                    den += (double)(src_ptr[j + n] * src_ptr[j + n]);
                }
            }
            result_ptr[j] = (float)(num / (sqrt(sum_templ)*sqrt(den)));
            num = 0.;
            den = 0.;
        }
    }
}

void FindPoints(Mat& srcImage , Mat& templImage , float scale)
{
    //cout<< "-----------"<<scale<<endl;
    int col=srcImage.cols;
    int row= srcImage.rows;
    resize(templImage,templImage,Size(templImage.cols*scale,templImage.rows*scale));
    resize(srcImage,srcImage,Size(srcImage.cols*0.5,srcImage.rows*0.5));
    resize(templImage,templImage,Size(templImage.cols*0.5,templImage.rows*0.5));


    std::chrono::steady_clock::time_point ncct1 = std::chrono::steady_clock::now();
    Mat resultMap;
    ncclong(srcImage, templImage, resultMap);
    //imshow("bb", resultMap);
    normalize(resultMap, resultMap, 0, 1,NORM_MINMAX);                 //一般归一化常用NORM_MINMAX，即将每个元素限制在一定范围内，但这个函数默认的模式是归一化二范数
    //imshow("bb", resultMap);
    std::chrono::steady_clock::time_point ncct2 = std::chrono::steady_clock::now();
    double ncctime= std::chrono::duration_cast<std::chrono::duration<double> >(ncct2 - ncct1).count();
    std::cout <<"NCC times = "<< ncctime << "s"<<endl;                                                            //只记录ncc算法所花费的时间，对于我用的模板和原图，耗时23秒左右


    double minVal, maxVal;
    Point minPos, maxPos, rightbottom;
    minMaxLoc(resultMap, &minVal, &maxVal, &minPos, &maxPos);
    maxPos = maxPos * 2;
    rightbottom.x=maxPos.x+templImage.cols*2;
    rightbottom.y=maxPos.y+templImage.rows*2;

    LeftUp.x = max(maxPos.x - 1,0);
    LeftUp.y = max(maxPos.y - 1,0);
    RightBottom.x = min( rightbottom.x , col - 1);
    RightBottom.y = min( rightbottom.y , row - 1);

    // cout <<"LeftUp point = "<< LeftUp<<"  |  "<< " RightBottom = "<< RightBottom <<endl;



    /************* show draw image ************/
/*
    resize(srcImage,srcImage,Size(srcImage.cols*2,srcImage.rows*2));
    resize(templImage,templImage,Size(templImage.cols*2,templImage.rows*2));
    rectangle(srcImage, maxPos, Point(maxPos.x + templImage.cols, maxPos.y + templImage.rows),Scalar(0,0,255));
    cv::circle(srcImage, maxPos, 5, cv::Scalar(0,255,0),2);
    cv::circle(srcImage, rightbottom, 5, cv::Scalar(0,255,255),2);
    char temp0[16];
    char temp1[16];
    sprintf(temp0, "(%d,%d)", maxPos.x, maxPos.y);
    sprintf(temp1, "(%d,%d)", rightbottom.x, rightbottom.y);
    Point text;
    text.x=rightbottom.x-100;
    text.y=rightbottom.y-20;
    putText(srcImage ,temp0, maxPos, CV_FONT_HERSHEY_COMPLEX, 1, Scalar( 255,0 ,255));
    putText(srcImage ,temp1, text, CV_FONT_HERSHEY_COMPLEX, 1, Scalar( 255,0 ,255));
    imshow("right",templImage);
    imshow("Fing BOX leftimg", srcImage);
    //imwrite("/home/jinln/000.png",srcImage);
    //waitKey((time)*1000);
*/
    return ;
}

}
