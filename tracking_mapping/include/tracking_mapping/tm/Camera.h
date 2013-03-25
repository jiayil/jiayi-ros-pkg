/* File:    Camera.h
   Author:  Jiayi Liu
   Date:    Mar 25, 2013
   Description:


  */
#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
using namespace cv;

class Camera
{
public:
    Camera();

    Mat K;
    Mat Kinv;

};
#endif // CAMERA_H
