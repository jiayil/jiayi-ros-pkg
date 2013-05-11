/* File:    Frame.cpp
   Author:  Jiayi Liu
   Date:    Apr 14, 2013
   Description:


  */
#include <stdio.h>
#include "tracking_mapping/cv/Frame.h"
#include "jlUtilities/opencv_helper.h"
Frame::Frame()
{
    frame_counter = 0;
    printf("Frame is constructed.\n");
}

void Frame::undistortFrame(Mat& frame, Mat& matIntrin, Mat& matDist)
{

//    IplImage* mx = cvCreateImage( Size(frame.cols, frame.rows), IPL_DEPTH_32F, 1 );
//    IplImage* my = cvCreateImage( Size(frame.cols, frame.rows), IPL_DEPTH_32F, 1 );

    Mat mx(frame.rows, frame.cols, CV_32FC1);
    Mat my(frame.rows, frame.cols, CV_32FC1);


    CvMat cvmatMx = mx;
    CvMat cvmatMy = my;

    CvMat cvmatIntrin = matIntrin;
    CvMat cvmatDist = matDist;

    jlUtilities::my_cvInitUndistortMap( &cvmatIntrin, &cvmatDist, &cvmatMx, &cvmatMy );

    CvMat cvmatFrame = frame;
    cvRemap( &cvmatFrame, &cvmatFrame, &cvmatMx, &cvmatMy); // undistort image

}

void Frame::buildFrameFromImage(Mat &img, Camera &cam)
{

    camera = cam;
//    undistortFrame(img, camera.mat_intrinsics, camera.mat_distCoeffs);
    mat_image = img.clone();

    cvtColor( mat_image, mat_image_gray, CV_BGR2GRAY );
    img_width = mat_image.cols;
    img_height = mat_image.rows;

    frame_counter++;




    vec_matchedFeaturePoints.clear();

    //-- CV
    detector.detect( mat_image_gray, vec_keypoints );
    extractor.compute( mat_image_gray, vec_keypoints, mat_descriptors );



}

void Frame::buildFrameFromImage(Mat &img)
{
    mat_image = img.clone();
    cvtColor( mat_image, mat_image_gray, CV_BGR2GRAY );
    img_width = mat_image.cols;
    img_height = mat_image.rows;
    frame_counter++;

    vec_matchedFeaturePoints.clear();

    //-- CV
    detector.detect( mat_image_gray, vec_keypoints );
    extractor.compute( mat_image_gray, vec_keypoints, mat_descriptors );

//    printf("Keypoints size: %d.\n", vec_keypoints.size());

}
