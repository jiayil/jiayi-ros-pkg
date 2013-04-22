/* File:    Frame.cpp
   Author:  Jiayi Liu
   Date:    Apr 14, 2013
   Description:


  */
#include <stdio.h>
#include "tracking_mapping/cv/Frame.h"

Frame::Frame()
{
    printf("Frame is constructed.\n");
}

void Frame::buildFrameFromImage(Mat &img, Camera &cam)
{
    mat_image = img.clone();
    cvtColor( mat_image, mat_image_gray, CV_BGR2GRAY );
    img_width = mat_image.cols;
    img_height = mat_image.rows;
    camera = cam;
    frame_counter++;

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


    //-- CV
    detector.detect( mat_image_gray, vec_keypoints );
    extractor.compute( mat_image_gray, vec_keypoints, mat_descriptors );

    printf("Keypoints size: %d.\n", vec_keypoints.size());

}
