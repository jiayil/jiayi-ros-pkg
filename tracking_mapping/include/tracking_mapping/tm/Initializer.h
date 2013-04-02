/* File:    Initializer.h
   Author:  Jiayi Liu
   Date:    Mar 25, 2013
   Description:


  */
#ifndef INITIALIZER_H
#define INITIALIZER_H

#include <opencv2/opencv.hpp>

#include "tracking_mapping/cv/FindCCC.h"
#include "tracking_mapping/cv/InterestPoints.h"
#include "tracking_mapping/cv/Pose.h"
#include "tracking_mapping/cv/PoseCCC.h"


class Initializer
{
public:
    Initializer();
    Initializer(std::string &name);

    bool process(cv::Mat &mat_image);

    FindCCC findCCC;
    PoseCCC poseCCC;
    InterestPoints interestPoints;

    int mode;
    const static float min_match_ratio = 0.5f;
    std::vector< cv::DMatch > matches;
    std::vector<cv::KeyPoint> vec_current_keypoints;

    // a list of descriptors of 100
    cv::Mat mat_poi_descriptors;
    // a list of descriptors of 5
    cv::Mat mat_roi;
    // Cordinates of the POIs on the object
    std::vector<cv::Point3f> vec_objCords;
    std::vector<cv::Point2d> vec_POIs;
};

#endif // INITIALIZER_H
