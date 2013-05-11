/* File:    Initializer.h
   Author:  Jiayi Liu
   Date:    Mar 25, 2013
   Description:


  */
#ifndef INITIALIZER_H
#define INITIALIZER_H

#include <opencv2/opencv.hpp>

#include "tracking_mapping/cv/Frame.h"
#include "tracking_mapping/cv/Pattern.h"

#include "tracking_mapping/cv/Dataset.h"

#include "tracking_mapping/cv/FindCCC.h"
#include "tracking_mapping/cv/InterestPoints.h"
#include "tracking_mapping/cv/Pose.h"
#include "tracking_mapping/cv/PoseCCC.h"


class Initializer
{
public:
    Initializer();


    bool process(cv::Mat &mat_image);
    void matchFeatures();
    bool computeHomography(Mat &H);
    void computePose();
    bool computePoseCorners(std::vector<cv::Point3f> &pts3D,
                            std::vector<cv::Point2f> &pts2D,
                            bool flag_useRANSAC);
    void computeLocation3D(std::vector<cv::Point2f> &pts2D,
                           std::vector<cv::Point3f> &pts3D,
                           cv::Point2f centerPoint,
                           float scale);


    bool processOnDataset(cv::Mat &mat_image);
    bool processOnDataset();
//    FindCCC findCCC;
//    PoseCCC poseCCC;
//    InterestPoints interestPoints;


    enum init_state_id
    {
        INIT_STATE_SUCCESS,     // Init'ed or pattern not found yet
        INIT_STATE_FAILED,      // Failed to init
        INIT_STATE_OBJ_FOUND,   // Pattern found
        INIT_STATE_POSE_COMPUTED// Pattern/camera pose computed
    };
    init_state_id current_state;

    int mode;
    size_t counter;

    Pattern pattern;
    Frame current_frame;

    Dataset dataset;

    const static float min_match_ratio = 0.6f;
    std::vector< cv::DMatch > matches;













};

#endif // INITIALIZER_H
