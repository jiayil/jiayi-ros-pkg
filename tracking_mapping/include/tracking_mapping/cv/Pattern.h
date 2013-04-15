/* File:    Pattern.h
   Author:  Jiayi Liu
   Date:    Apr 14, 2013
   Description:


  */
#ifndef PATTERN_H
#define PATTERN_H
#include <opencv2/opencv.hpp>

#include "tracking_mapping/cv/Frame.h"
#include "tracking_mapping/cv/Camera.h"

class Pattern : public Frame
{
public:
    Pattern();

    enum pattern_state_id
    {
        PATTERN_STATE_SUCCESS,  // Init'ed
        PATTERN_STATE_FAILED    // Failed to init
    };
    pattern_state_id current_state;

    //-- Physical
    std::vector<cv::Point2f> vec_corners;   // four corners of the book: TL TR BR BL
    std::vector<cv::Point2f> vec_corners_currentImg;
    double obj_width, obj_height;

    //-- CV
    Point obj_img_center;
    std::vector<Point3f> vec_obj_corners3d;


};

#endif // PATTERN_H
