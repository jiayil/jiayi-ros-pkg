/* File:    CvKeySwitch.cpp
   Author:  Jiayi Liu
   Date:    Mar 31, 2013
   Description:


  */

#include <stdio.h>
#include "jlUtilities/CvKeySwitch.h"


namespace jlUtilities
{
CvKeySwitch::CvKeySwitch()
{

}

void CvKeySwitch::state(char key, CvImageWriter *ptrCIW)
{
    switch(key)
    {
//    case 27:
//        current_state = TM_STATE_EXIT;
//        printf("\nTM_STATE_EXIT\n");
//        break;
//    case 't':
//        if(current_state == TM_STATE_TRACKING)
//        {
//            current_state = TM_STATE_READY;
//            text_state = "Tracking: Off";
//            color_state.val[0] = 255;
//            color_state.val[1] = 0;
//            printf("\nTM_STATE_READY\n");
//        }
//        else if(current_state == TM_STATE_READY)
//        {
//            current_state = TM_STATE_TRACKING;
//            text_state = "Tracking: On";
//            color_state.val[0] = 0;
//            color_state.val[1] = 255;
//            printf("\nTM_STATE_TRACKING\n");
//        }
//        break;
//    case 's':
////        String fileName = fileName_prefix + boost::lexical_cast<string>(fileName_numbering) + fileName_suffix;
////        imwrite(fileName, frame_gray);
////        fileName_numbering++;
////        printf("%s is saved\n", fileName.c_str());
    }
}



}
