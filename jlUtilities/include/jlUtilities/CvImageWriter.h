/* File:    CvImageWriter.h
   Author:  Jiayi Liu
   Date:    Mar 31, 2013
   Description:


  */

#ifndef CVIMAGEWRITER_H
#define CVIMAGEWRITER_H

#include <string>
#include <opencv2/opencv.hpp>

namespace jlUtilities
{
class CvImageWriter
{
public:
    CvImageWriter(std::string name_prefix = "");
    int writeImage(cv::Mat &image);


    std::string fileName_prefix;
    std::string fileName_suffix;
    unsigned int fileName_numbering;
};



}
#endif // CVIMAGEWRITER_H
