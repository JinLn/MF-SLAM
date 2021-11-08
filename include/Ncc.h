#ifndef NCC
#define NCC

#include <iostream>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui.hpp>
#include<vector>
#include "Frame.h"
using namespace cv;

namespace ORB_SLAM3
{

        // static Point* upleft ,* boright;

        void ncclong(Mat& srcImage, Mat& templImage, Mat& result);

        void FindPoints(Mat& srcImage , Mat& templImage , float scale);

}

#endif
