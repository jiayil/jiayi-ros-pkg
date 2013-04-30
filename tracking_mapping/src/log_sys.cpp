/* File:    log_sys.cpp
   Author:  Jiayi Liu
   Date:    Apr 28, 2013
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
    ros::NodeHandle node;

    ros::Subscriber sub_cmdUpdateClam = node.subscribe("/cmd_clam", 10, &cmdClamCallback);






    Mat frame;
    char key;
    enum sys_state_id {
        SYS_STATE_EXIT,
        SYS_STATE_READY,
        SYS_STATE_TRACKING,
        SYS_STATE_PAUSE,
        SYS_STATE_SLIDE
    };
    sys_state_id current_state = SYS_STATE_READY;

    VideoCapture cap;
    std::string val = argv[1];
    std::string fileName, filePrefix, fileSuffix;
    int fileNum, fileInterval;
    int timeWaitKey = 30;


    if(argc == 2 && val == "0" )
    {
        fileName = "Camera";
        cap.open(1);
        cap.set(CV_CAP_PROP_FRAME_WIDTH, 800);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, 600);
        current_state = SYS_STATE_READY;
    }
    else if(argc == 2 && val == "1" )
    {
        fileName = "data/metric/ad_0_0_770_0.jpg";
        frame = imread(fileName);
        current_state = SYS_STATE_PAUSE;
    }
    else if(argc == 5)
    {
        filePrefix = argv[1];
        fileNum = boost::lexical_cast<int>( argv[2] );
        fileInterval = boost::lexical_cast<int>( argv[3] );
        fileSuffix = argv[4];

        fileName = filePrefix +
                boost::lexical_cast<std::string>(fileNum) +
                fileSuffix;
        frame = imread(fileName);

        timeWaitKey = 0;
        current_state = SYS_STATE_SLIDE;
    }
    else
    {
        fileName = "data/VideoTest.avi";
        cap.open(fileName);
        current_state = SYS_STATE_READY;
    }

    if(frame.data == NULL && current_state != SYS_STATE_READY)
    {
        printf("%s not found.\n", fileName.c_str());
        return -1;
    }

    if(!cap.isOpened() && current_state == SYS_STATE_READY)  // check if we succeeded
    {
        printf("%s not found.\n", fileName.c_str());
        return -1;
    }
//    cap.set(CV_CAP_PROP_FRAME_WIDTH, 800);
//    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 600);
//    VideoWriter writer("VideoTest.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25.0, Size(800, 600));


    while (ros::ok())
    {


        if(current_state == SYS_STATE_READY)
        {
            cap >> frame;

//            printf("size: %dx%d\n", frame.rows, frame.cols);

            if(frame.empty())
            {
                printf("Video ends.\n");
                break;
            }

        }
        else if(current_state == SYS_STATE_SLIDE && key == '=')
        {
            fileName = filePrefix +
                    boost::lexical_cast<std::string>(fileNum) +
                    fileSuffix;
            frame = imread(fileName);
            fileNum += fileInterval;

            printf("Read %s\n", fileName.c_str());

            if(frame.data == NULL)
            {
                printf("%s not found.\n", fileName.c_str());
                break;
            }
        }
        imshow("video", frame);
        cvbridgeImg.header.frame_id = fileNum;
        cvbridgeImg.header.stamp = ros::Time::now();
        cvbridgeImg.image = frame;

        pub.publish(cvbridgeImg.toImageMsg());


        key = waitKey(timeWaitKey);
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
        else if(key == 'n')
        {
            cap >> frame;

            if(frame.empty())
            {
                printf("Video ends.\n");
                break;
            }
        }
    }
    std::cout<< "Finish playing." <<std::endl;
    return 0;
}
