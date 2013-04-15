/* File:    CvImageWriter.cpp
   Author:  Jiayi Liu
   Date:    Mar 31, 2013
   Description:


  */

#include <stdio.h>
#include <boost/lexical_cast.hpp>

#include "jlUtilities/CvImageWriter.h"

namespace jlUtilities
{
CvImageWriter::CvImageWriter(std::string name_prefix)
{
    if(name_prefix == "")
        fileName_prefix = "image_";
    else
        fileName_prefix = fileName_prefix;

    fileName_numbering = 0;
    fileName_suffix = ".jpg";
}

int CvImageWriter::writeImage(cv::Mat &image)
{
    std::string fileName =
            fileName_prefix +
            boost::lexical_cast<std::string>(fileName_numbering) +
            fileName_suffix;

    imwrite(fileName, image);
    fileName_numbering++;
    printf("%s is saved.\n", fileName.c_str());

    return 0;
}



}
