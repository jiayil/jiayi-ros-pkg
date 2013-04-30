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
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <cv_bridge/cv_bridge.h>


using namespace cv;

nav_msgs::Path path_cam;
void camPathCallback(const nav_msgs::Path& msg)
{
  path_cam = msg;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "broadcast_video");
    ros::NodeHandle nh;
    tf::Transform tfCamGroundTruth;
    nav_msgs::Path pathCam_;
    geometry_msgs::PoseStamped poseStampedCam_;
    ros::Subscriber sub_path_cam_ = nh.subscribe("path_cam", 10, &camPathCallback);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/image_raw", 1);
    cv_bridge::CvImage cvbridgeImg;
    cvbridgeImg.encoding = "bgr8";

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
    if(argc == 1)
    {
        std::cout<< "Usage: ./broadcast_video num" << std::endl;
        std::cout<< "num: 0 - camera"<< std::endl
                 << "     1 - video" << std::endl
                 << "     2 - image" << std::endl;
        return -1;
    }
    std::string val = argv[1];
    std::string fileName, filePrefix, fileSuffix;
    int fileNum, fileInterval;

    double transArray[6];

    int timeWaitKey = 30;




    if(argc == 2 && val == "0" )
    {
        fileName = "Camera";
        cap.open(1);
        cap.set(CV_CAP_PROP_FRAME_WIDTH, 800);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, 600);
        current_state = SYS_STATE_READY;
    }
    else if(argc == 2 && val == "2" )
    {
        fileName = "data/metric/ad_0_0_770_0.jpg";
        frame = imread(fileName);
        current_state = SYS_STATE_PAUSE;
    }
    else if(argc ==2 && val == "1")
    {
        fileName = "data/VideoTest.avi";

        cap.open(fileName);

        current_state = SYS_STATE_READY;
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

        transArray[0] = 0;
        transArray[1] = 0;
        transArray[2] = 770;
        transArray[3] = 0;
        transArray[4] = 0;
        transArray[5] = 0;

        timeWaitKey = 0;
        current_state = SYS_STATE_SLIDE;
    }
    else
    {
        fileName = "data/VideoTest.avi";
        printf("ok0\n");
        cap.open(fileName);
        printf("ok\n");
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
