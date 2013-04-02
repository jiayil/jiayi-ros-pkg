
/*
  File:   jlUtilities.cpp
  Author: Jiayi Liu
  Date:   Feb 25, 2012
  Description:
 */
#include <unistd.h>
#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl/common/angles.h>

#include "jlUtilities/jlUtilities.h"


namespace jlUtilities
{
  
  void tryLookUpTF(tf::TransformListener &listener,
                   std::string &target_frame, std::string &source_frame,
                   tf::StampedTransform &tf, double duration)
  {
    try
    {
      listener.waitForTransform(target_frame, source_frame,  
                                ros::Time(0), ros::Duration(duration));
      listener.lookupTransform(target_frame, source_frame,  
                               ros::Time(0), tf);
      
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }
    
  }
  
  void printTF(tf::Transform transform, std::string str)
  {
    tf::Matrix3x3 rotMatrix = transform.getBasis();
    tf::Vector3 originVector = transform.getOrigin();

    
    tfScalar roll, pitch, yaw;
    
    
    
    
    rotMatrix.getRPY(roll, pitch, yaw);
    
    
    ROS_INFO("TF %s (x y z r p y): %f %f %f %f %f %f", 
             str.c_str(),
             originVector.x(),
             originVector.y(), 
             originVector.z(),
             pcl::rad2deg(roll),
             pcl::rad2deg(pitch),
             pcl::rad2deg(yaw));
  }
  
  void printTF(tf::StampedTransform &transform, std::string str)
  {
    tf::Matrix3x3 rotMatrix = transform.getBasis();
    tf::Vector3 originVector = transform.getOrigin();

    
    tfScalar roll, pitch, yaw;
    
    
    
    
    rotMatrix.getRPY(roll, pitch, yaw);
    
    
    ROS_INFO("TF %s (x y z r p y): %f %f %f %f %f %f", 
             str.c_str(),
             originVector.x(),
             originVector.y(), 
             originVector.z(),
             pcl::rad2deg(roll),
             pcl::rad2deg(pitch),
             pcl::rad2deg(yaw));
  }
  
  void printPose(geometry_msgs::Pose &pose, int mode)
  {
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.orientation, q);
    tf::Matrix3x3 rotMatrix = tf::Matrix3x3(q);
    switch(mode)
    {
    case 0:
      tfScalar roll, pitch, yaw;
      
      
      
      
      rotMatrix.getRPY(roll, pitch, yaw);
      
      
      ROS_INFO("Pose elements (x y z r p y): %f %f %f %f %f %f", 
               pose.position.x,
               pose.position.y, 
               pose.position.z,
               pcl::rad2deg(roll),
               pcl::rad2deg(pitch),
               pcl::rad2deg(yaw));
      break;
    case 1:
      

      

      
      ROS_INFO("Pose matrix:");
      printf("%8.3f %8.3f %8.3f %8.3f\n%8.3f %8.3f %8.3f %8.3f\n%8.3f %8.3f %8.3f %8.3f\n%8.3f %8.3f %8.3f %8.3f\n",
             rotMatrix.getRow(0).getX(), rotMatrix.getRow(0).getY(), rotMatrix.getRow(0).getZ(), pose.position.x,
             rotMatrix.getRow(1).getX(), rotMatrix.getRow(1).getY(), rotMatrix.getRow(1).getZ(), pose.position.y,
             rotMatrix.getRow(2).getX(), rotMatrix.getRow(2).getY(), rotMatrix.getRow(2).getZ(), pose.position.z,
             0.0, 0.0, 0.0, 1.0);
    break;
    case 2:
    
      ROS_INFO("Pose x: %f, y: %f, z: %f, yaw: %f", 
               pose.position.x,
               pose.position.y, 
               pose.position.z,
               tf::getYaw(pose.orientation));
    break;
  }
  }
  
  void printPose(geometry_msgs::PoseStamped &pose, int mode)
  {
    printPose(pose.pose, mode);
  }
  
  
  void poseStampedToStampedTF(geometry_msgs::PoseStamped &msg, tf::StampedTransform &stampedTF, std::string child)
  {
    tf::Transform btTrans;
    stampedTF.stamp_ = msg.header.stamp;
    stampedTF.frame_id_ = msg.header.frame_id;
    stampedTF.child_frame_id_ = child;
    
    tf::poseMsgToTF(msg.pose, btTrans);
    stampedTF.setData(btTrans);
  }
  
  void stampedTFToPoseStamped(tf::StampedTransform &stampedTF, geometry_msgs::PoseStamped &msg)
  {
    msg.header.stamp = stampedTF.stamp_;
    msg.header.frame_id = stampedTF.frame_id_;
    
    tf::Vector3 translation = stampedTF.getOrigin();
    
    msg.pose.position.x = translation.x();
    msg.pose.position.y = translation.y();
    msg.pose.position.z = translation.z();
    
    tf::quaternionTFToMsg(stampedTF.getRotation(), msg.pose.orientation);
               
  }
  
  
  bool calibAprilTag(geometry_msgs::PoseStamped &pose)
  {
    double dist = pose.pose.position.x;
    if(dist < 0.8)
      pose.pose.position.z -= 0.03;
    else if(dist < 1.13)
      pose.pose.position.z -= 0.02;
    else if(dist < 1.54)
      pose.pose.position.z -= 0.01;
    else if(dist < 1.85)
      pose.pose.position.z -= 0.00;
    else if(dist < 2.19)
      pose.pose.position.z -= -0.01;
    else if(dist < 2.54)
      pose.pose.position.z -= -0.02;
    else if(dist < 2.91)
      pose.pose.position.z -= -0.03;
    else if(dist < 3.0)
      pose.pose.position.z -= -0.04;
    else
      return false;
    
    return true;
    
  }
  


  
  void splitFilePath(const std::string &str, std::string &fileFolder, std::string &fileName)
  {
    size_t found;
    found=str.find_last_of("/\\");
    fileFolder = str.substr(0,found);
    fileName = str.substr(found+1);
  }
  
  
  void updateFileName (std::string& str, std::string newName)
  {    
    size_t found;

    found = str.find_last_of(".");
    if( newName == "")
    {
      newName = str.substr(0,found) + "_new" + str.substr(found+1);
    }
    else
    {
      newName += str.substr(found+1);
    }
    
    str = newName;

  }
  
  void refinePose(geometry_msgs::PoseStamped &pose, tf::StampedTransform &transform, std::string &child)
  {
    printPose(pose);
    tf::StampedTransform tfPose;
    poseStampedToStampedTF(pose, tfPose, child);
    tfPose *= transform;
    
    stampedTFToPoseStamped(tfPose, pose);
    printPose(pose);
  }
  
  void stampedTFDifferenceStampedTF(tf::StampedTransform &tf1,
                                    tf::StampedTransform &tf2,
                                    tf::StampedTransform &tfResult)
  {
    tfResult.setData(tf1.inverseTimes(tf2));

  }
  
  void tfStampedDifferenceTF(geometry_msgs::TransformStamped &tfMsg1,
                             geometry_msgs::TransformStamped &tfMsg2,
                             tf::StampedTransform &tfResult)
  {
    tf::StampedTransform tf1, tf2;
    tf::transformStampedMsgToTF(tfMsg1, tf1);
    tf::transformStampedMsgToTF(tfMsg2, tf2);
    stampedTFDifferenceStampedTF(tf1, tf2, tfResult);
    
    
  }
  
  // the result tf is relative to pose1
  void poseStampedDifferenceTF(geometry_msgs::PoseStamped &pose1,
                               geometry_msgs::PoseStamped &pose2,
                               tf::StampedTransform &tfResult)
  {
    tf::StampedTransform tf1, tf2;
    poseStampedToStampedTF(pose1, tf1);
    poseStampedToStampedTF(pose2, tf2);
    stampedTFDifferenceStampedTF(tf1, tf2, tfResult);
  }
  
  void foutAppendTF(tf::StampedTransform &transform, std::ofstream &fout)
  {
    tf::Matrix3x3 rotMatrix = transform.getBasis();
    tf::Vector3 originVector = transform.getOrigin();

    
    tfScalar roll, pitch, yaw;
    
    
    
    
    rotMatrix.getRPY(roll, pitch, yaw);
    
    fout<< originVector.x() << " "
        << originVector.y() << " "
        << originVector.z() << " "
        << pcl::rad2deg(roll) << " "
        << pcl::rad2deg(pitch) << " "
        << pcl::rad2deg(yaw) << std::endl;
    
    
  }
  void foutAppendTF(tf::Transform &transform, std::ofstream &fout)
  {
    tf::StampedTransform stf;
    stf.setData(transform);
    foutAppendTF(stf, fout);
    
  }
  
  void foutAppendPose(geometry_msgs::PoseStamped &pose, std::ofstream &fout)
  {
    tf::StampedTransform transform;
    
    poseStampedToStampedTF(pose, transform);
    
    
    tf::Matrix3x3 rotMatrix = transform.getBasis();
    tf::Vector3 originVector = transform.getOrigin();

    
    tfScalar roll, pitch, yaw;
    
    
    
    
    rotMatrix.getRPY(roll, pitch, yaw);
    
    fout<< originVector.x() << " "
        << originVector.y() << " "
        << originVector.z() << " "
        << pcl::rad2deg(roll) << " "
        << pcl::rad2deg(pitch) << " "
        << pcl::rad2deg(yaw) << std::endl;
    
    
  }
  
  void poseStampedToBtscalarArray(geometry_msgs::PoseStamped &pose, tfScalar arr[])
  {
    tfScalar roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.pose.orientation, q);
    tf::Matrix3x3 rotMatrix = tf::Matrix3x3(q);
    rotMatrix.getRPY(roll, pitch, yaw);
    
    arr[0] = (tfScalar)pose.pose.position.x;
    arr[1] = (tfScalar)pose.pose.position.y;
    arr[2] = (tfScalar)pose.pose.position.z;
    
    arr[3] = (tfScalar)pcl::rad2deg(roll);
    arr[4] = (tfScalar)pcl::rad2deg(pitch);
    arr[5] = (tfScalar)pcl::rad2deg(yaw);
    
    
  }
      
  void appendBtscalarArrayToString(tfScalar arr[], std::string &str)
  {
    std::stringstream strStream;
    
    strStream.precision(2);
    
    for(int i=0;i<6;i++)
      strStream << arr[i] << "_";
      
    str = strStream.str();
    str.erase(str.size()-1);
  }
  
  void foutAppendTFStamped(geometry_msgs::TransformStamped &transform, 
                           std::ofstream &fout)
  {
    
    tf::Quaternion q;
    tf::quaternionMsgToTF(transform.transform.rotation, q);
    tf::Matrix3x3 rotMatrix = tf::Matrix3x3(q);
    
    tfScalar roll, pitch, yaw;
    
    
    
    
    rotMatrix.getRPY(roll, pitch, yaw);
    
    fout<< transform.transform.translation.x << " "
        << transform.transform.translation.y << " "
        << transform.transform.translation.z << " "
        << pcl::rad2deg(roll) << " "
        << pcl::rad2deg(pitch) << " "
        << pcl::rad2deg(yaw) << std::endl;
    
    
  }
  
  void tfToPose(tf::StampedTransform &stampedTF, geometry_msgs::Pose &msg)
  {

    
    tf::Vector3 translation = stampedTF.getOrigin();
    
    msg.position.x = translation.x();
    msg.position.y = translation.y();
    msg.position.z = translation.z();
    
    tf::quaternionTFToMsg(stampedTF.getRotation(), msg.orientation);
    
  }
  void eigenMatrix4fToTransform(Eigen::Matrix4f &m, tf::Transform &t)
  {
    tf::Matrix3x3 basis = tf::Matrix3x3(m(0,0), m(0,1), m(0,2),
                                    m(1,0), m(1,1), m(1,2),
                                    m(2,0), m(2,1), m(2,2));
    tf::Vector3   origin = tf::Vector3(m(0,3), m(1,3), m(2,3));
    t.setBasis(basis);
    t.setOrigin(origin);
    
    
  }
  
  tf::Transform eigenMatrix4fToTransform(Eigen::Matrix4f &m)
  {
    tf::Transform t;
        
    tf::Matrix3x3 basis = tf::Matrix3x3(m(0,0), m(0,1), m(0,2),
                                    m(1,0), m(1,1), m(1,2),
                                    m(2,0), m(2,1), m(2,2));
    tf::Vector3   origin = tf::Vector3(m(0,3), m(1,3), m(2,3));
    t.setBasis(basis);
    t.setOrigin(origin);
    
    return t;
  }
  
  void transformToEigenMatrix4f(tf::Transform &t, Eigen::Matrix4f &m)
  {
    tf::Matrix3x3 basis;
    tf::Vector3   origin, rot[3];
    basis = t.getBasis();
    origin = t.getOrigin();
    
    
    for(int i=0;i<3;i++)
      rot[i] = basis.getRow(i);
    
    m << rot[0][0], rot[0][1], rot[0][2], origin.x(),
         rot[1][0], rot[1][1], rot[1][2], origin.y(),
         rot[2][0], rot[2][1], rot[2][2], origin.z(),
         0, 0, 0, 1;
  }
  Eigen::Matrix4f transformToEigenMatrix4f(tf::Transform &t)
  {
    Eigen::Matrix4f m;
        
    tf::Matrix3x3 basis;
    tf::Vector3   origin, rot[3];
    basis = t.getBasis();
    origin = t.getOrigin();
    
    
    for(int i=0;i<3;i++)
      rot[i] = basis.getRow(i);
    
    m << rot[0][0], rot[0][1], rot[0][2], origin.x(),
         rot[1][0], rot[1][1], rot[1][2], origin.y(),
         rot[2][0], rot[2][1], rot[2][2], origin.z(),
         0, 0, 0, 1;
    
    return m;
  }
  
  void tfToPose(tf::Transform &trans, geometry_msgs::PoseStamped &msg)
  {
    tf::quaternionTFToMsg(trans.getRotation(), msg.pose.orientation);
    msg.pose.position.x = trans.getOrigin().x();
    msg.pose.position.y = trans.getOrigin().y();
    msg.pose.position.z = trans.getOrigin().z();
  }
  
  void profilingLoggerCurrentTimeDuration(ros::Time currentTime,
                                          ros::Time startTime,
                                          LogThis &logger,
                                          std::string durationName)
  {
    logger.log(currentTime.toNSec(), "currentTime");
    logger.log((currentTime-startTime).toSec(), durationName);
  }
  

  void loadXYZRPYs(std::string &filePath,
                   std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &tArray)
  {
    std::ifstream fin(filePath.c_str());
    
    tf::Transform t;
    
    float trans[6];
    
    while(!fin.eof( ))
    {
      for(int i=0;i<6;i++)
        fin >> trans[i];
      for(int i=3;i<6;i++)
        trans[i] = pcl::deg2rad(trans[i]);
      
      t.setOrigin(tf::Vector3(trans[0], trans[1], trans[2]));
      t.setRotation(tf::createQuaternionFromRPY(trans[3], trans[4], trans[5]));
        
      tArray.push_back(transformToEigenMatrix4f(t));
    }
        
    std::cout<< "Loading " << tArray.size() << " matrix completed" << std::endl;
    
  }


  
}
