/* File:    Camera.h
   Author:  Jiayi Liu
   Date:    Mar 25, 2013
   Description:


  */
#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>


class Camera
{
public:
    Camera();
    Camera& operator=(const Camera& cam);

    cv::Mat mat_intrinsics, mat_extrinsics;
    cv::Mat mat_distCoeffs;
    cv::Mat mat_transform;  // in obj coords


    cv::Mat K;

};
#endif // CAMERA_H
