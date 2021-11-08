#include <iostream>
#include <vector>
#include <string>
#include "include/ORBextractor.h"
#include <opencv2/opencv.hpp>
#include "Drawkp.h"
#include "Points.h"
#include <string>
using namespace cv;
using namespace std;


void DrawKP(cv::Mat& dstImg, std::vector<cv::KeyPoint>& vKpts)
{
    const float radius = 3.0f;
    const bool showmod=false;
    //cv::resize(dstImg,dstImg,cv::Size(dstImg.cols, dstImg.rows));
    if(showmod){
        // 遍历所有的特征点,并且使用绿色圆圈绘制出来
        for(const auto& kp : vKpts)
        {
            auto layer = kp.octave;
            float factor = pow(0.8f, layer+1);
            //float fMyRaidus = radius / factor;
            float fMyRaidus = radius ;
            cv::circle( dstImg,kp.pt,fMyRaidus, cv::Scalar(0,255,0));

            // 绘制方向
            int x = round(fMyRaidus * sin(kp.angle) + kp.pt.x);
            int y = round(fMyRaidus * cos(kp.angle) + kp.pt.y);
            cv::line(dstImg, kp.pt,cv::Point2i(x,y),cv::Scalar(0,0,255));
        }
    }
    else{
        //使用作者方式显示特征点
        const float r=5;
        const int n = vKpts.size();

        for(int i=0;i<n;i++)
        {
            //在特征点附近正方形选择四个点
            cv::Point2f pt1,pt2;
            pt1.x=vKpts[i].pt.x-r;
            pt1.y=vKpts[i].pt.y-r;
            pt2.x=vKpts[i].pt.x+r;
            pt2.y=vKpts[i].pt.y+r;
            cv::rectangle(dstImg,pt1,pt2,cv::Scalar(0,255,0));
            cv::circle(dstImg,vKpts[i].pt,2,cv::Scalar(0,255,0),-1);
        }//遍历所有的特征点

    }
    return ;
}

//打印matched特征点
void DrawKP(cv::Mat& dstImg, std::vector<cv::KeyPoint>& vKpts ,std::vector<float>& mvuflag )
{
    const float radius = 3.0f;
    const bool showmod=false;
    //cv::resize(dstImg,dstImg,cv::Size(dstImg.cols, dstImg.rows));
    if(showmod){
        // 遍历所有的特征点,并且使用绿色圆圈绘制出来
        for(const auto& kp : vKpts)
        {
            auto layer = kp.octave;
            float factor = pow(0.8f, layer+1);
            //float fMyRaidus = radius / factor;
            float fMyRaidus = radius ;
            cv::circle( dstImg,kp.pt,fMyRaidus, cv::Scalar(0,255,0));

            // 绘制方向
            int x = round(fMyRaidus * sin(kp.angle) + kp.pt.x);
            int y = round(fMyRaidus * cos(kp.angle) + kp.pt.y);
            cv::line(dstImg, kp.pt,cv::Point2i(x,y),cv::Scalar(0,0,255));
        }
    }
    else{
        //使用作者方式显示特征点
        const float r=5;
        const int n = vKpts.size();

        for(int i=0 ; i<n ; i++)
        {
            if(mvuflag[i]!=-1){//打印特征点
            //在特征点附近正方形选择四个点
            cv::Point2f pt1,pt2;
            pt1.x=vKpts[i].pt.x-r;
            pt1.y=vKpts[i].pt.y-r;
            pt2.x=vKpts[i].pt.x+r;
            pt2.y=vKpts[i].pt.y+r;
            cv::rectangle(dstImg,pt1,pt2,cv::Scalar(0,255,0));
            cv::circle(dstImg,vKpts[i].pt,2,cv::Scalar(0,255,0),-1);
            }
        }//遍历所有的特征点

    }
    return ;
}
//cvCircle(CvArr* img, CvPoint center, int radius, CvScalar color, int thickness=1, int lineType=8, int shift=0)
//thickness 如果是正数，表示组成圆的线条的粗细程度，否则，表示圆是否被填充；
//line_type 线条的类型，默认是8；
//shift 圆心坐标点和半径值的小数点位数。

//绘制匹配点
void DrawMatch(cv::Mat& imgRes, std::vector<KeyPoint>& vKpRes1, std::vector<KeyPoint>& vKpRes2, std::vector<float>& vMatchedKptsIdin1 ,std::vector<float> vMatchedKptsIdin2)
{
    char name[150]={};
    sprintf(name,"/home/jinln/match/%06d.png",ORB_SLAM3::squenceimg);
    // step 1 首先在两张图像上绘制提取得到的特征点
    //const float     fRadius = 1.0f;
    const size_t    nNumKpts1 = vKpRes1.size();
    //cv::rectangle(imgRes, ORB_SLAM3::LeftUp, ORB_SLAM3::RightBottom, cv::Scalar(0,255,0) ,2);//边框
    // CV_EXPORTS_W void putText( InputOutputArray img, const String& text, Point org,int fontFace, double fontScale, Scalar color,int thickness = 1, int lineType = LINE_8, bool bottomLeftOrigin = false );
    char temp0[16];
    char temp1[16];
    sprintf(temp0, "(%d,%d)", ORB_SLAM3::LeftUp.x, ORB_SLAM3::LeftUp.y);
    sprintf(temp1, "(%d,%d)", ORB_SLAM3::RightBottom.x, ORB_SLAM3::RightBottom.y);
    Point tx ,ty;
    tx.x= ORB_SLAM3::LeftUp.x + 5; tx.y= ORB_SLAM3::LeftUp.y + 20;
    ty.x= ORB_SLAM3::RightBottom.x - 105 ; ty.y = ORB_SLAM3::RightBottom.y - 5;
    //putText(imgRes , temp0 , tx , 1, 1.2, cv::Scalar(0,255,255) ,1);//顶点
    //putText(imgRes , temp1 , ty, 1, 1.2, cv::Scalar(0,255,255) ,1);
    // 绘制图像1上的特征点
    const float r=5;
    for(size_t nKpt1Id = 0; nKpt1Id < nNumKpts1; ++nKpt1Id)
    {
        //const KeyPoint& kpt = vKpRes1[nKpt1Id];

        cv::Point2f pt1,pt2;
        pt1.x=vKpRes1[nKpt1Id].pt.x-r;
        pt1.y=vKpRes1[nKpt1Id].pt.y-r;
        pt2.x=vKpRes1[nKpt1Id].pt.x+r;
        pt2.y=vKpRes1[nKpt1Id].pt.y+r;

        //float fPtRadius = fRadius / pow(0.8f, kpt.octave+1);
        if(vMatchedKptsIdin1[nKpt1Id] == -1){

            //cv::rectangle(imgRes,pt1,pt2,cv::Scalar(0,255,0));
            //cv::circle(imgRes,vKpRes1[nKpt1Id].pt,2,cv::Scalar(0,255,0));
            //cv::circle(imgRes, kpt.pt,fPtRadius, cv::Scalar(0,255,0));
        }
        else {
            cv::rectangle(imgRes,pt1,pt2,cv::Scalar(0,0,255));
            cv::circle(imgRes,vKpRes1[nKpt1Id].pt,2,cv::Scalar(0,0,255));
            //cv::circle(imgRes, kpt.pt,fPtRadius, cv::Scalar(0,0,255),2);
        }

    }

    // 绘制图像2上的特征点
    const size_t    nNumKpts2 = vKpRes2.size();
    // 绘制图像2坐标点的时候需要使用的偏移
    const float rr=3;
    const cv::Point2f ptOffset(static_cast<float>(imgRes.cols>>1),0.0f);
    for(size_t nKpt2Id = 0; nKpt2Id < nNumKpts2; ++nKpt2Id)
    {
        //const KeyPoint& kpt = vKpRes2[nKpt2Id];

        cv::Point2f pt1,pt2;
        pt1.x=vKpRes2[nKpt2Id].pt.x-r;
        pt1.y=vKpRes2[nKpt2Id].pt.y-r;
        pt2.x=vKpRes2[nKpt2Id].pt.x+r;
        pt2.y=vKpRes2[nKpt2Id].pt.y+r;

        //float fPtRadius = fRadius / pow(0.8f, kpt.octave+1);
        if( vMatchedKptsIdin2[nKpt2Id] == -1)
        {

            //cv::rectangle(imgRes,pt1+ptOffset,pt2+ptOffset,cv::Scalar(0,255,0));
            //cv::circle(imgRes,vKpRes2[nKpt2Id].pt+ptOffset,2,cv::Scalar(0,255,0));
            //cv::circle(imgRes, kpt.pt + ptOffset,fPtRadius, cv::Scalar(0,255,255));
        }
        else {
            cv::rectangle(imgRes,pt1+ptOffset,pt2+ptOffset,cv::Scalar(0,0,255));
            cv::circle(imgRes,vKpRes2[nKpt2Id].pt+ptOffset,2,cv::Scalar(0,0,255));
            //cv::circle(imgRes, kpt.pt + ptOffset,fPtRadius, cv::Scalar(0,0,255),2);
        }
    }

    int matchednums=0;
    bool flag = true;//draw line
    if(flag){
    // 绘制匹配关系
        for(size_t nKpt1Id = 0; nKpt1Id < nNumKpts1 ; ++nKpt1Id)
            {
                const KeyPoint& kpt1 = vKpRes1[nKpt1Id];

                // 如果没有匹配关系就不绘制
                if(vMatchedKptsIdin1[nKpt1Id] == -1)    continue;

                else
                {
                    // 如果有匹配关系就获取另外一个点
                    const KeyPoint& kpt2 = vKpRes2[vMatchedKptsIdin1[nKpt1Id]];

                    // 绘制连线
                    int red=rand()%255;
                    int blue=rand()%255;
                    int green=rand()%255;
                    if(nKpt1Id%16==0)
                    cv::line(imgRes, kpt1.pt, kpt2.pt + ptOffset, cv::Scalar(blue,green,red));
                    matchednums++;
                }
            }
    }
       //cout<<"DrawMatch matchednums= "<<matchednums<<endl;
       copyMakeBorder( imgRes, imgRes, 0, 60, 0, 0, BORDER_CONSTANT, Scalar( 0, 0, 0));
       putText(imgRes ,"Matched Points Numbers: ", Point(10, ORB_SLAM3::height+25) , CV_FONT_HERSHEY_COMPLEX, 1, Scalar( 255,255 ,255));
       putText(imgRes ,std::to_string(matchednums), Point(475,ORB_SLAM3::height+25), CV_FONT_HERSHEY_COMPLEX, 1, Scalar( 255, 255, 255));
       putText(imgRes ,"Matching Consumption Time: ", Point(650, ORB_SLAM3::height+25) , CV_FONT_HERSHEY_COMPLEX, 1, Scalar( 255,255 ,255));
       putText(imgRes ,std::to_string(ORB_SLAM3::duration), Point(1180,ORB_SLAM3::height+25), CV_FONT_HERSHEY_COMPLEX, 1, Scalar( 255, 255, 255));
       putText(imgRes ,"PreAver Consumption Time: ", Point(10, ORB_SLAM3::height+55) , CV_FONT_HERSHEY_COMPLEX, 1, Scalar( 255,255 ,255));
       putText(imgRes ,std::to_string(ORB_SLAM3::averduration), Point(500,ORB_SLAM3::height+55), CV_FONT_HERSHEY_COMPLEX, 1, Scalar( 255, 255, 255));
       //imwrite(name,imgRes);
       if(imgRes.rows>720)
           resize(imgRes,imgRes,Size(imgRes.cols/2,imgRes.rows/2));
       //imshow(" Draw match Keypoints connected ",imgRes);
       return ;
}

void ShowKeypoints(Mat& img1, Mat& img2, string& S){

        Size sz1 = img1.size();
        Size sz2 = img2.size();
        Mat im3(sz1.height, sz1.width+sz2.width, CV_8UC3);
        Mat left(im3, Rect(0, 0, sz1.width, sz1.height));
        img1.copyTo(left);
        Mat right(im3, Rect(sz1.width, 0, sz2.width, sz2.height));
        img2.copyTo(right);
        imshow(S, im3);
        //waitKey(10);
        return;
}

void ShowKeypoints(Mat& img1, Mat& img2, Mat& img3){

        Size sz1 = img1.size();
        Size sz2 = img2.size();
        Mat im3(sz1.height, sz1.width+sz2.width, img1.type());
        Mat left(im3, Rect(0, 0, sz1.width, sz1.height));
        img1.copyTo(left);
        Mat right(im3, Rect(sz1.width, 0, sz2.width, sz2.height));
        img2.copyTo(right);
        //imshow(S, im3);
        im3.copyTo(img3);
        im3.release();
        //waitKey(10);
        return;
}

void printMat(Mat& matprint){
    cout<<"--------------------------------------------------------------------------------" << endl;
    for (int i = 0; i< matprint.rows; i++){
        for (int j = 0; j< matprint.cols; j++)
        {
            cout << setw(20) <<matprint.at<float>(i, j);
        }
        cout<< endl;
    }
    cout<<"--------------------------------------------------------------------------------" << endl;
}

void myremapimg(Mat& img ,Mat& K ,Mat& D,Mat& R, Mat& P){
    cv::Mat lmapx, lmapy, imgU;
    cv::initUndistortRectifyMap(K, D, R, P, img.size(), CV_32F, lmapx, lmapy);
    cv::remap(img, img, lmapx, lmapy, cv::INTER_LINEAR);
    return ;
}
