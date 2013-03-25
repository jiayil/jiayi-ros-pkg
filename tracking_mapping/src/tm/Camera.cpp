/* File:    Camera.cpp
   Author:  Jiayi Liu
   Date:    Mar 25, 2013
   Description:


  */

#include <cstdio>
#include "tracking_mapping/tm/Camera.h"

Camera::Camera()
{
    // Camera intrinsic matrix (assume no lens dist)
    double K_[3][3] =
    { {881.68473, 0, 398.14820},
    {0,  884.03796, 289.57924},
    {0,   0,   1} };

    K = cv::Mat(3, 3, CV_64F, K_).clone();
    Kinv = K.inv();
    printf("Camera constructed.\n");
}
