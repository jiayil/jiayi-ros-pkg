/* File:    opencv_helper.cpp
   Author:  Jiayi Liu
   Date:    Mar 17, 2013
   Description:


  */

#include "jlUtilities/opencv_helper.h"

namespace jlUtilities
{
bool sort_feature_response(const KeyPoint &first, const KeyPoint &second)
{
    return first.response > second.response;
}
}
