/* File:    Camera.cpp
   Author:  Jiayi Liu
   Date:    Mar 25, 2013
   Description:


  */

#include <cstdio>
#include "tracking_mapping/cv/Camera.h"

Camera::Camera()
{
    // Camera intrinsic matrix (assume no lens dist)
//    double K_[3][3] =
//    { {881.68473, 0, 398.14820},
//    {0,  884.03796, 289.57924},
//    {0,   0,   1} };

//    K = cv::Mat(3, 3, CV_64F, K_).clone();

    printf("Camera constructed.\n");
}

Camera& Camera::operator =(const Camera& cam)
{
    mat_intrinsics = cam.mat_intrinsics.clone();
    mat_extrinsics = cam.mat_extrinsics.clone();
    mat_distCoeffs = cam.mat_distCoeffs.clone();
    mat_transform  = cam.mat_transform.clone();
    K = cam.K.clone();
}
