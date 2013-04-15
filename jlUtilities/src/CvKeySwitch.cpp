/* File:    CvKeySwitch.cpp
   Author:  Jiayi Liu
   Date:    Mar 31, 2013
   Description:


  */

#include <stdio.h>
#include "jlUtilities/CvKeySwitch.h"


namespace jlUtilities
{
CvKeySwitch::CvKeySwitch(CvImageWriter *ptrCIW)
{
    if(ptrCIW == NULL)
        ptr_image_writer = &image_writer_;
    else
        ptr_image_writer = ptrCIW;
    printf("CvKeySwitch constructed.\n");

}

int CvKeySwitch::switchState(char key, cv::Mat *ptr_image)
{
    int retValue = 0;
    switch(key)
    {
    case 27:
        current_state = SYS_STATE_EXIT;
        std::cout<< "---------------" << std::endl
                 << "SYS_STATE_EXIT"  << std::endl
                 << "---------------" << std::endl;
        break;
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
    case 's':
        if(ptr_image == NULL)
        {
            printf("Error: No image.\n");
            retValue = -1;
            break;
        }
        ptr_image_writer->writeImage(*ptr_image);

        break;
    }

    return retValue;
}





}
