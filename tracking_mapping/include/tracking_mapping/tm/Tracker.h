/* File:    Tracker.h
   Author:  Jiayi Liu
   Date:    Mar 17, 2013
   Description:


  */
#ifndef TRACKER_H
#define TRACKER_H

#include <opencv2/opencv.hpp>

#include "tracking_mapping/tm/Camera.h"
#include "tracking_mapping/tm/Initializer.h"

using namespace cv;
class Tracker
{
public:
    Tracker();
    bool init();
    int updateFrame();
    void drawOpticalFlowWithCurrent();
    void getFundamentalMat(Mat &F,
                           std::vector<cv::KeyPoint> keypoints1,
                           std::vector<cv::KeyPoint> keypoints2,
                           std::vector<DMatch> matches);
//    void getCameraMat(const Mat& K,
//                      const Mat& Kinv,
//                      const vector<KeyPoint>& imgpts1,
//                      const vector<KeyPoint>& imgpts2,
//                      Matx34d& P,
//                      Matx34d& P1,
//                      vector<DMatch>& matches,
//                      vector<CloudPoint>& outCloud
//                      );

    Mat mat_image_current;
    Mat mat_image_previous;
    Mat mat_keyImage_current;
    Mat mat_keyImage_previous;

    Mat mat_image_canvas;


    bool flag_first_image;
    bool flag_initialized;


    std::vector<DMatch> of_matches;
    std::vector<cv::KeyPoint> vecKeypointsPrevious, vecKeypointsCurrent;

    std::vector<uchar> of_status;
    std::vector<float> of_error;

    Camera camera;
    Initializer initializer;

    std::string text_state;



};

#endif // TRACKER_H
