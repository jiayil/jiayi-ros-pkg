/* File:    myutils.h
   Author:  Jiayi Liu
   Date:    Mar 25, 2013
   Description:


  */
#ifndef MYUTILS_H
#define MYUTILS_H


#include <opencv2/opencv.hpp>
#include "tracking_mapping/cv/Pose.h"
#include <iostream>

using namespace cv;

namespace myutils
{
static const double pi = 3.14159265358979323846;
inline static double square(int a)
{
return a * a;
}

// Scale an image to 0..255 and put it into an image of type CV_8U
void scaleImageTo8U(cv::Mat& imageInput, cv::Mat& imageOutput);

// Scale an image to 0..255 and put it into an image of type CV_8UC3
void scaleImageTo8UC3(cv::Mat& imageInput, cv::Mat& imageOutput);

// Print a matrix
void printMat(cv::Mat& m);

// Draw model coordinate axes
void drawAxes(cv::Mat& imageInput, cv::Mat& K, Pose pose);

void putStatus(cv::Mat &img, std::string & str);

}



#endif // MYUTILS_H
