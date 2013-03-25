/* File:    System.cpp
   Author:  Jiayi Liu
   Date:    Mar 17, 2013
   Description:


  */

#include <ros/callback_queue.h>
#include "tracking_mapping/tm/System.h"
#include "tracking_mapping/tm/myutils.h"

System::System()
{
    image_transport::ImageTransport it(nh_);
    std::string topic = std::string("/image_raw");
    sub_image_ = it.subscribe(topic,
                              1,
                              &System::imageCallback,
                              this,
                              image_transport::TransportHints("compressed", ros::TransportHints().tcpNoDelay(true)));
    pub_image_preview_ = it.advertise("image_preview", 1);
    printf("System constructed.\n");
}

void System::run()
{
    ros::MultiThreadedSpinner spinner(0); // Default: 0 - One thread for each CPU core
    spinner.spin(); // spin() will not return until the node has been shutdown


//    ros::Rate r(15); // 10 hz
//    while (ros::ok())
//    {

//      ros::spinOnce();
//      r.sleep();
//    }

}

void System::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    // Prepare images
    // Seperate ROS framework and this tracking/mapping framework
    // Keep ROS stuff here in System
    {
        // convert ros-msg to tracker's cv::mat
        cv_bridge::CvImagePtr cv_ptr;

        // First image of the system
        if(tracker_.flag_first_image)
        {
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
                //ROS_INFO("%s", cv_ptr->encoding.c_str());
    //            tracker_.mat_image_previous = cv_ptr->image.clone();
    //            tracker_.mat_keyImage_previous = cv_ptr->image.clone();
                tracker_.mat_image_current = cv_ptr->image.clone();

                FastFeatureDetector ffd;
                ffd.detect(tracker_.mat_image_previous, tracker_.vecKeypointsPrevious);

            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            tracker_.flag_first_image = false;
            return;
        }

        // All the other images
        try
        {
            tracker_.mat_image_previous = tracker_.mat_image_current.clone();
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            //ROS_INFO("%s", cv_ptr->encoding.c_str());
            tracker_.mat_image_current = cv_ptr->image.clone();
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Prepare canvas as current image for future plotting
        tracker_.mat_image_canvas = tracker_.mat_image_current.clone();

    }

    // Initialize the tracker with a known object
    if(!tracker_.flag_initialized)
    {
//        printf("init\n");
        if(tracker_.init() == true)
        {
            tracker_.flag_initialized = true;
        }
    }
    else    // Tracker is initialized
    {

myutils::putStatus(tracker_.mat_image_canvas,
                   tracker_.text_state);


        // Do optical flow
//        tracker_.updateFrame();
//    tracker_.drawOpticalFlowWithCurrent();

    //    Mat F;
    //    tracker_.getFundamentalMat(F,
    //                               tracker_.vecKeypointsPrevious,
    //                               tracker_.vecKeypointsCurrent,
    //                               tracker_.of_matches);
    //    std::cout << F << std::endl;


    }




    // Show canvas
    image_test.header = msg->header;
    image_test.encoding = "bgr8";
    image_test.image = tracker_.mat_image_canvas;


    pub_image_preview_.publish(image_test.toImageMsg());


}
