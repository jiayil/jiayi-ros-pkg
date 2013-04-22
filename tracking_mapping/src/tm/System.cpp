/* File:    System.cpp
   Author:  Jiayi Liu
   Date:    Mar 17, 2013
   Description:


  */

#include <ros/callback_queue.h>


#include "tracking_mapping/tm/System.h"
#include "tracking_mapping/tm/myutils.h"

#include "jlUtilities/jlUtilities.h"


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

    pub_odom_cam_ = nh_.advertise<nav_msgs::Odometry>("odom_cam", 1);
    pub_path_cam_ = nh_.advertise<nav_msgs::Path>("path_cam", 1);

    tfOpticalInLocal.setOrigin(  tf::Vector3(0.0, 0.0, 0.0) );
    tfOpticalInLocal.setRotation( tf::Quaternion(-0.5, 0.5, -0.5, 0.5) );
//    tfOpticalInLocal = tfOpticalInLocal.inverse();


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

    if(tracker_.current_state == tracker_.TRCK_STATE_INITING)
    {
        // 1: a book cover
        // 0: Dr. Hoff's pattern
        if(tracker_.init(1) == true)
        {
            tracker_.current_state = tracker_.TRCK_STATE_GOOD;
        }
        else if(tracker_.initializer.current_state ==
                tracker_.initializer.INIT_STATE_POSE_COMPUTED)
        {
//            tfCam.setOrigin(  tf::Vector3(tracker_.vec_trans[0],
//                                          tracker_.vec_trans[1],
//                                          tracker_.vec_trans[2]) );
//            tfCam.setRotation( tf::createQuaternionFromRPY(tracker_.vec_rot[0],
//                                                           tracker_.vec_rot[1],
//                                                           tracker_.vec_rot[2]) );
//            br.sendTransform(tf::StampedTransform(tfCam, ros::Time::now(), "world", "cam"));



            Mat M = tracker_.camera.mat_transform;

            tfCam.setOrigin(  tf::Vector3(M.at<double>(0, 3),
                                          M.at<double>(1, 3),
                                          M.at<double>(2, 3) ));


            btMatrix3x3 rot;
            rot.setValue(M.at<double>(0, 0), M.at<double>(0, 1), M.at<double>(0, 2),
                         M.at<double>(1, 0), M.at<double>(1, 1), M.at<double>(1, 2),
                         M.at<double>(2, 0), M.at<double>(2, 1), M.at<double>(2, 2));
//            rot = rot.inverse();
            btQuaternion q;
            rot.getRotation(q);

            tfCam.setRotation( q );
            tfCam = tfCam.inverse();
//            tfCam *= tfOpticalInLocal;
            ros::Time t = ros::Time::now();
            br.sendTransform(tf::StampedTransform(tfCam, t, "world", "cam"));

            //-- Pub odom
            jlUtilities::tfToPose(tfCam, poseStampedCam_);
            poseStampedCam_.header.stamp = t;
            poseStampedCam_.header.frame_id = "world";

            odomCam_.pose.pose = poseStampedCam_.pose;
            odomCam_.header.stamp = poseStampedCam_.header.stamp;
            odomCam_.header.frame_id = poseStampedCam_.header.frame_id;
            odomCam_.child_frame_id = "cam";

            pub_odom_cam_.publish(odomCam_);

            //-- Pub path

            pathCam_.header = poseStampedCam_.header;
            pathCam_.poses.push_back(poseStampedCam_);
            pub_path_cam_.publish(pathCam_);

        }

    }
    else if(tracker_.current_state == tracker_.TRCK_STATE_GOOD)
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
    else if(tracker_.current_state == tracker_.TRCK_STATE_FAILED)
    {
        myutils::putStatus(tracker_.mat_image_canvas,
                           tracker_.text_state);
        printf("%s\r", tracker_.text_state.c_str());

    }






    // Show canvas
    image_test.header = msg->header;
    image_test.encoding = "bgr8";
    image_test.image = tracker_.mat_image_canvas;


    pub_image_preview_.publish(image_test.toImageMsg());


}
