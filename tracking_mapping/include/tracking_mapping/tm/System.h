/* File:    System.h
   Author:  Jiayi Liu
   Date:    Mar 17, 2013
   Description:


  */

#ifndef SYSTEM_H
#define SYSTEM_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


#include <opencv2/opencv.hpp>

#include "tracking_mapping/tm/Tracker.h"

class System
{
public:
    System();
    void run();
private:
    ros::NodeHandle nh_;
    image_transport::Subscriber sub_image_;
    image_transport::Publisher pub_image_preview_;

    Tracker tracker_;

    cv_bridge::CvImage image_test;

    void imageCallback(const sensor_msgs::ImageConstPtr & msg);

};
#endif // SYSTEM_H
