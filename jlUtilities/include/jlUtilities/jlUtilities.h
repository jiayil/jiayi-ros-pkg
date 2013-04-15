/*
  File:   jlUtilities.h
  Author: Jiayi Liu
  Date:   Feb 25, 2012
  Description:

 */
#ifndef JLUTILITIES_H
#define JLUTILITIES_H
#include <fstream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>

#include "jlUtilities/logThis.h"

#include <Eigen/StdVector>

namespace jlUtilities
{

void tryLookUpTF(tf::TransformListener &listener,
                 std::string &target_frame, std::string &source_frame,
                 tf::StampedTransform &tf, double duration = 3.0);
void printTF(tf::Transform transform, std::string str = std::string());
void printTF(tf::StampedTransform &transform, std::string str = std::string());
void printPose(geometry_msgs::PoseStamped &pose, int mode = 0);
void printPose(geometry_msgs::Pose &pose, int mode = 0);

void poseStampedToStampedTF(geometry_msgs::PoseStamped &msg, tf::StampedTransform &stampedTF, std::string child = std::string());
void stampedTFToPoseStamped(tf::StampedTransform &stampedTF, geometry_msgs::PoseStamped &msg);

bool calibAprilTag(geometry_msgs::PoseStamped &pose);
  template <typename PointT>
  int loadPointCloudFromPCD(const std::string &file_name, pcl::PointCloud<PointT> &cloud)
  {
        
    if (pcl::io::loadPCDFile<PointT> (file_name, cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file %s \n", file_name.c_str());
      return (-1);
    }
    std::cout << "Loaded "
        << cloud.width * cloud.height
        << " data points from test_pcd.pcd with the following fields: "
        << pcl::getFieldsList(cloud)
        << std::endl;
    return 0;
  }
  template <typename PointT>
  int savePointCloudToPCD(const std::string &pcdFileName, pcl::PointCloud<PointT> &cloud, std::string who)
  {
    ///////////////////////////////////////////////////////////// 
    // write cloud to file
    pcl::io::savePCDFileASCII (pcdFileName, cloud);
    std::cout<< who << " saved " << cloud.points.size() << " data points to " << pcdFileName << std::endl;
    
    return 0;
  }

void splitFilePath(const std::string &str, std::string &fileFolder, std::string &fileName);
void updateFileName (const std::string& str, std::string newName = std::string());

void refinePose(geometry_msgs::PoseStamped &pose, tf::StampedTransform &transform, std::string &child);

void stampedTFDifferenceStampedTF(tf::StampedTransform &tf1,
                                  tf::StampedTransform &tf2,
                                  tf::StampedTransform &tfResult);

void tfStampedDifferenceTF(geometry_msgs::TransformStamped &tfMsg1,
                           geometry_msgs::TransformStamped &tfMsg2,
                           tf::StampedTransform &tfResult);


// the result tf is relative to pose1
void poseStampedDifferenceTF(geometry_msgs::PoseStamped &pose1,
                             geometry_msgs::PoseStamped &pose2,
                             tf::StampedTransform &tfResult);

void foutAppendTF(tf::StampedTransform &transform, std::ofstream &fout);
void foutAppendTF(tf::Transform &transform, std::ofstream &fout);
void foutAppendPose(geometry_msgs::PoseStamped &pose, std::ofstream &fout);

void poseStampedToBtscalarArray(geometry_msgs::PoseStamped &pose, tfScalar arr[]);
void appendBtscalarArrayToString(tfScalar arr[], std::string &str);

void foutAppendTFStamped(geometry_msgs::TransformStamped &transform, 
                         std::ofstream &fout);

void tfToPose(tf::StampedTransform &stampedTF, geometry_msgs::Pose &msg);
void tfToPose(tf::Transform &trans, geometry_msgs::PoseStamped &msg);
void tfToPose(tf::StampedTransform &stampedTF, geometry_msgs::PoseStamped &msg);
void eigenMatrix4fToTransform(Eigen::Matrix4f &m, tf::Transform &t);
tf::Transform eigenMatrix4fToTransform(Eigen::Matrix4f &m);
void transformToEigenMatrix4f(tf::Transform &t, Eigen::Matrix4f &m);
Eigen::Matrix4f transformToEigenMatrix4f(tf::Transform &t);

void profilingLoggerCurrentTimeDuration(ros::Time currentTime,
                                        ros::Time startTime,
                                        LogThis &logger,
                                        std::string durationName = std::string("duration"));

void loadXYZRPYs(std::string &filePath, 
                 std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &tArray);

}
#endif
