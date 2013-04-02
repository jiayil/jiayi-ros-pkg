/*
  File:   logThis.cpp
  Author: Jiayi Liu
  Date:   Feb 25, 2012
  Description:

 */





#include "jlUtilities/jlUtilities.h"
#include "jlUtilities/logThis.h"

using namespace jlUtilities;


LogThis::LogThis(std::string fileName, int noteWidth)
{
  this->width = noteWidth;
  this->fileName = fileName;
  fout = new std::ofstream(fileName.c_str());
  ROS_INFO("File %s created", fileName.c_str());
}
LogThis::~LogThis()
{
  delete fout;
  ROS_INFO("File %s closed", fileName.c_str());    
}

//void LogThis::stop()
//{
//  this->fout.flush();
//  this->fout.close();
//}
void LogThis::log(std::string note)
{
  *fout<< std::setw(width) << std::left << note << std::endl;
}

void LogThis::log(int seq, std::string note)
{
  *fout<< std::setw(width) << std::left << note;
  *fout<< seq << std::endl;  
}

void LogThis::log(uint32_t seq, std::string note)
{
  *fout<< std::setw(width) << std::left << note;
  *fout<< seq << std::endl;  
}

void LogThis::log(ros::Time t, std::string note)
{
  *fout<< std::setw(width) << std::left << note;
  *fout<< t.toNSec() << std::endl;  
}

void LogThis::log(double t, std::string note)
{
  *fout<< std::setw(width) << std::left << note;
  *fout<< t << std::endl;  
}

void LogThis::log(uint64_t t, std::string note)
{
  *fout<< std::setw(width) << std::left << note;
  *fout<< t << std::endl;  
}

void LogThis::log(tf::StampedTransform &transform, std::string note)
{
  *fout<< std::setw(width) << std::left << note;
  foutAppendTF(transform, *fout);
}

void LogThis::log(tf::Transform &transform, std::string note)
{
  *fout<< std::setw(width) << std::left << note;
  foutAppendTF(transform, *fout);
}
void LogThis::log(geometry_msgs::PoseStamped &pose, std::string note)
{

  *fout<< std::setw(width) << std::left << note;
  foutAppendPose(pose, *fout);
}



