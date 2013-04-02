/* File:   logThis.h
   Author: Jiayi Liu
   Date:   Feb 25, 2012
   Description:

 */
#ifndef LOGTHIS_H
#define LOGTHIS_H

#include <fstream>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>

namespace jlUtilities
{
  class LogThis
  {
    int width;
    std::string fileName;
    std::ofstream *fout;
    
  public:
    LogThis(std::string fileName, int noteWidth = 20);
    ~LogThis();

    
//    void init();
//    void stop();
    void log(std::string note);
    void log(int seq, std::string note);
    void log(uint32_t seq, std::string note);
    
    void log(ros::Time t, std::string note);
    void log(double t, std::string note);
    void log(uint64_t t, std::string note);
    void log(tf::StampedTransform &transform, std::string note);
    void log(tf::Transform &transform, std::string note);
    void log(geometry_msgs::PoseStamped &pose, std::string note);

  };
}
#endif // LOGTHIS_H
