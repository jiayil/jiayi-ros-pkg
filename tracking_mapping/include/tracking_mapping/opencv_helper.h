/* File:    opencv_helper.h
   Author:  Jiayi Liu
   Date:    Mar 17, 2013
   Description:


  */
#ifndef OPENCV_HELPER_H
#define OPENCV_HELPER_H

#include <opencv2/opencv.hpp>
using namespace cv;

namespace opencv_helper
{
static const double pi = 3.14159265358979323846;
inline static double square(int a)
{
return a * a;
}

}


#endif // OPENCV_HELPER_H
