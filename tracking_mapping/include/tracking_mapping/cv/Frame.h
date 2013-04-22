/* File:    Frame.h
   Author:  Jiayi Liu
   Date:    Apr 14, 2013
   Description:


  */
#ifndef FRAME_H
#define FRAME_H
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include "tracking_mapping/cv/Camera.h"

using namespace cv;

class Frame
{
public:
    Frame();
    void buildFrameFromImage(Mat &img, Camera &cam);
    void buildFrameFromImage(Mat &img);

    //-- Physical
    Mat mat_image;
    Mat mat_image_gray;
    int img_width, img_height;
    size_t frame_counter;    //!!! Need to add a function to check boundary
    // can't be filled in buildFrameFromImage()
    Camera camera;

    //-- CV

    SurfFeatureDetector detector;
    FREAK extractor;
//    SurfDescriptorExtractor extractor;
    std::vector<KeyPoint> vec_keypoints;
    std::vector<cv::Point2f> vec_matchedFeaturePoints;   // point datatype from KeyPoint, matched with another frame
    Mat mat_descriptors;
    // can't be filled in buildFrameFromImage()
    float scale;


};

#endif // FRAME_H
