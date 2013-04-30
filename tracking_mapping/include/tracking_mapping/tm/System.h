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
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>

#include "tracking_mapping/tm/Tracker.h"

#include <ctime>

class System
{
public:
    System();
    void run();
private:
    ros::NodeHandle nh_;
    image_transport::Subscriber sub_image_;
    image_transport::Publisher pub_image_preview_;

    ros::Publisher pub_odom_cam_;
    ros::Publisher pub_path_cam_;


    tf::Transform tfCam;    // obj in cam
    tf::Transform tfCamGroundTruth;
    tf::Transform tfOpticalInLocal, tfObjInWorld;
    tf::TransformBroadcaster br;
    nav_msgs::Odometry odomCam_;
    nav_msgs::Path pathCam_;
    geometry_msgs::PoseStamped poseStampedCam_;

    Tracker tracker_;

    cv_bridge::CvImage image_test;

    //-- Profiling
    clock_t start_time, finish_time;
    double elapsed_time;
    double fps;         // keep a running avg for frames per sec

    void imageCallback(const sensor_msgs::ImageConstPtr & msg);

};
#endif // SYSTEM_H
