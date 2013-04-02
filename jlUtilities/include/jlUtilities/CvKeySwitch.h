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
    enum tm_state_id {
        TM_STATE_EXIT,
        TM_STATE_READY,
        TM_STATE_TRACKING
    };
    tm_state_id current_state;


    CvKeySwitch();

    void state(char key, CvImageWriter *ptrCIW = NULL);
};




}
#endif // CVKEYSWITCH_H
