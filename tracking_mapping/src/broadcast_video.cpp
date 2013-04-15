/* File:    broadcast_video.cpp
   Author:  Jiayi Liu
   Date:    Apr 13, 2013
   Description:


  */

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

using namespace cv;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "broadcast_video");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/image_raw", 1);
    cv_bridge::CvImage cvbridgeImg;
    cvbridgeImg.encoding = "bgr8";

    VideoCapture cap("data/VideoTest.avi");
    if(!cap.isOpened())  // check if we succeeded
    {
        printf("File not found.\n");
        return -1;
    }
//    cap.set(CV_CAP_PROP_FRAME_WIDTH, 800);
//    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 600);
//    VideoWriter writer("VideoTest.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25.0, Size(800, 600));
    Mat frame;
    char key;
    enum sys_state_id {
        SYS_STATE_EXIT,
        SYS_STATE_READY,
        SYS_STATE_TRACKING,
        SYS_STATE_PAUSE
    };
    sys_state_id current_state = SYS_STATE_READY;

    while (1)
    {
        key = waitKey(30);
        if ( key == 27)
        {
            break;
        }
        else if(key == 'p') // pause
        {
            if(current_state == SYS_STATE_READY)
                current_state = SYS_STATE_PAUSE;
            else if(current_state == SYS_STATE_PAUSE)
                current_state = SYS_STATE_READY;

        }
        if(current_state == SYS_STATE_READY)
        {
            cap >> frame;

            if(frame.empty())
            {
                printf("Video ends.\n");
                break;
            }

        }
        imshow("video", frame);
        cvbridgeImg.header.stamp = ros::Time::now();
        cvbridgeImg.image = frame;

        pub.publish(cvbridgeImg.toImageMsg());



    }
    std::cout<< "Finish playing." <<std::endl;
    return 0;
}
