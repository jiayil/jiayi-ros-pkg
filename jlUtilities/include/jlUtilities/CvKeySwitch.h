/* File:    CvKeySwitch.h
   Author:  Jiayi Liu
   Date:    Mar 31, 2013
   Description:


  */

#ifndef CVKEYSWITCH_H
#define CVKEYSWITCH_H

#include <string>

#include "jlUtilities/CvImageWriter.h"

namespace jlUtilities
{
class CvKeySwitch
{
    CvKeySwitch(CvImageWriter *ptrCIW = NULL);


    enum sys_state_id {
        SYS_STATE_EXIT,
        SYS_STATE_READY,
        SYS_STATE_TRACKING,
        SYS_STATE_PAUSE
    };
    sys_state_id current_state;

    CvImageWriter *ptr_image_writer;

    int switchState(char key, cv::Mat *ptr_image = NULL);

private:
    CvImageWriter image_writer_;

};




}
#endif // CVKEYSWITCH_H
