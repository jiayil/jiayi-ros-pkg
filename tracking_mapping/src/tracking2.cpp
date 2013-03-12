
/* File:    tracking2.cpp
   Author:  Jiayi Liu
   Date:
   Description:
            Rich feature based tracking on successive frames.

  */

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <boost/lexical_cast.hpp>


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#define DEBUG
#ifdef DEBUG
# define DEBUG_PRINT(x) printf x
#else
# define DEBUG_PRINT(x) do {} while (0)
#endif

using namespace cv;


enum state_id {
    STATE_EXIT,
    STATE_READY,
    STATE_TRACKING
};

typedef struct
{
    bool tracked;
    int trackedIDinPool;
}tblTracked_elem_t;

bool sort_feature_response(const KeyPoint &first, const KeyPoint &second)
{
    return first.response > second.response;
}

Mat mat_ros_frame;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      //ROS_INFO("%s", cv_ptr->encoding.c_str());
      mat_ros_frame = cv_ptr->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "tracking2");
    ros::NodeHandle node;
    ros::Subscriber sub;
    ros::Publisher pub_frame = node.advertise<sensor_msgs::Image>("/image_feature", 50);
    cv_bridge::CvImage img_for_pub;

    bool use_ros = true;

    if(argc == 2)
        use_ros = false;
    else
    {
        sub = node.subscribe("/image_raw", 10, &imageCallback);
    }
    String fileName_prefix = "image_";
    String fileName_suffix = ".jpg";
    int fileName_numbering = 0;

    int num_trackingPoints = 50;
    char keyIn;
//    std::string FEATURE_NAME = "SIFT";
    state_id current_state = STATE_READY;

    int num_vecKeypoints, num_vecKeypoints_new;

    int minHessian = 1200;
    SurfFeatureDetector detector( minHessian );
    //    Ptr<FeatureDetector> detector = FeatureDetector::create(FEATURE_NAME);
    std::vector<KeyPoint> vecKeypoints;
    std::vector<KeyPoint> vecKeypoints_new;

    SurfDescriptorExtractor extractor;
//    OrbDescriptorExtractor extractor;
    Mat matDescriptors, matDescriptors_new;
    Mat matDescriptorPool;
    FlannBasedMatcher matcher;
    std::vector<DMatch> matches;

    float min_match_ratio = 0.5f;

    VideoCapture cap;
    if(use_ros == false)
    {
        cap.open(1);
        if(!cap.isOpened())  // check if we succeeded
            return -1;
        cap.set(CV_CAP_PROP_FRAME_WIDTH, 800);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, 600);
    }



    // Use "y" to show that the baseLine is about
    std::string text_state = "Tracking: Off";
    Scalar color_state = Scalar(255, 0, 0);
    int fontFace = FONT_HERSHEY_SIMPLEX;
    double fontScale = 1;
    int thickness = 2;
    double fontScale_feature = 0.5;
    Point2f fontShift_feature(5,0);

    int baseline=0;
    int cnt_wait_for_ros_msg = 10;

    namedWindow("Feature",1);
    namedWindow("Feature1",1);
    int cnt_test = 20;
    bool flag_cnt_test = true;
    ros::Rate r(20);


    while(ros::ok())
    {
        ros::spinOnce();
        Mat frame, frame_new;
        if(use_ros == false)
        {
            cap.grab();
            cap.grab();
            cap.grab();
            cap.retrieve(frame);

            //cap >> frame; // get a new frame from camera
            //printf("CAM FPS: %f\n", cap.get(CV_CAP_PROP_FPS));
        }
        else
            frame = mat_ros_frame.clone();





        ros::spinOnce();

        frame_new = mat_ros_frame.clone();


        if(frame.data == NULL)
        {
            if(cnt_wait_for_ros_msg--<0)
            {
                ROS_INFO("Waiting time exceeded. Exit");
                return -1;
            }
            waitKey(30);
            continue;
        }

        Mat frame_gray, frame_new_gray;

        cvtColor( frame, frame_gray, CV_BGR2GRAY );
        cvtColor( frame_new, frame_new_gray, CV_BGR2GRAY );
        //DEBUG_PRINT(("Current frame Dim: %d; Col: %d; Row: %d\n", frame.dims, frame.cols, frame.rows));


        if(current_state == STATE_EXIT)
        {
            break;
        }
        else if(current_state == STATE_TRACKING)
        {
/*
             Feature tracking and special cases:
             * No feature in current frame
             * Features found in current frame:
                * New empty pool
                * Initialized pool:
                    * All features in current frame are new and untracked
                    * Only add untracked features into pool
*/
            if(!frame_gray.data)
            {
                printf(" --(!)Error reading video frames\n");
                return -1;
            }
            detector.detect(frame_gray, vecKeypoints);
            detector.detect(frame_new_gray, vecKeypoints_new);

            num_vecKeypoints = vecKeypoints.size();
            num_vecKeypoints_new = vecKeypoints_new.size();
            if(num_vecKeypoints == 0 || num_vecKeypoints_new == 0)
            {
                printf("No feature detected\n");
            }
            else
            {
                printf("Detected: %d features.\n", num_vecKeypoints);

                // Sort Keypoints according to "response"
                std::sort(vecKeypoints.begin(), vecKeypoints.end(), sort_feature_response);
                std::sort(vecKeypoints_new.begin(), vecKeypoints_new.end(), sort_feature_response);

                if(num_vecKeypoints > num_trackingPoints)
                {
                    num_vecKeypoints = num_trackingPoints;
                    vecKeypoints.erase(vecKeypoints.begin() + num_vecKeypoints, vecKeypoints.end());
                }
                if(num_vecKeypoints_new > num_trackingPoints)
                {
                    num_vecKeypoints_new = num_trackingPoints;
                    vecKeypoints_new.erase(vecKeypoints_new.begin() + num_vecKeypoints_new, vecKeypoints_new.end());
                }
                DEBUG_PRINT(("Length: %d\n", vecKeypoints.size()));
                extractor.compute( frame_gray, vecKeypoints, matDescriptors );
                extractor.compute( frame_new_gray, vecKeypoints_new, matDescriptors_new );

                DEBUG_PRINT(("Current Descriptors Dim: %d; Col: %d; Row: %d\n", matDescriptors.dims, matDescriptors.cols, matDescriptors.rows));

                if(matDescriptorPool.rows == 0)
                {
                    matDescriptorPool.push_back(matDescriptors);
                    for(int i = 0; i<vecKeypoints.size(); i++)
                    {
                        circle(frame, vecKeypoints[i].pt, 7, Scalar(0, 255, 0), 3);
                        putText(frame, boost::lexical_cast<string>(i), vecKeypoints[i].pt+fontShift_feature,
                                fontFace, fontScale_feature, Scalar(0, 255, 0), 1, 8);
                    }
                }
                else
                {
                    matches.clear();
                    double dist_match_max = 0;
                    int    id_dist_match_max;

                    std::vector<std::vector< DMatch > > knnMatches;
                    matcher.knnMatch(matDescriptors, matDescriptorPool, knnMatches, 2 );
                    DEBUG_PRINT(("knnMatch num: %d\n", knnMatches.size()));

                    // KNN match will return 2 nearest
                    // matches for each query descriptor
                    for (int i=0; i<knnMatches.size(); i++)
                    {
                        const cv::DMatch& bestMatch = knnMatches[i][0];
                        const cv::DMatch& betterMatch = knnMatches[i][1];
                        float distanceRatio = bestMatch.distance / betterMatch.distance;

                        if(distanceRatio < min_match_ratio)
                        {
                            matches.push_back(bestMatch);
                        }

                        if(bestMatch.distance > dist_match_max)
                        {
                            dist_match_max = bestMatch.distance;
                            id_dist_match_max = i;
                        }
                    }
                    DEBUG_PRINT(("Match num: %d\n", matches.size()));

                    DEBUG_PRINT(("dist_match_max: %f\n", dist_match_max));
                    circle(frame, vecKeypoints[knnMatches[id_dist_match_max][0].queryIdx].pt, 10, Scalar(0, 255, 255), 3);

                    // Add new untracked keypoints to Pool
                    if(matches.size() != 0)
                    {
                        // Init a Tracked/Untracked LookUp Table (LUT) corresponding to current 'matDescriptors'
                        tblTracked_elem_t tblTracked_elem;
                        tblTracked_elem.tracked = false;
                        tblTracked_elem.trackedIDinPool = 0;
                        std::vector<tblTracked_elem_t> tblTracked(matDescriptors.rows, tblTracked_elem);

                        // Go through 'matches' to label the LUT above with
                        // True:    tracked
                        // False:   untracked
                        for (std::vector<DMatch>::iterator it = matches.begin(); it != matches.end(); ++it)
                        {
                            tblTracked[(*it).queryIdx].tracked = true;
                            tblTracked[(*it).queryIdx].trackedIDinPool = (*it).trainIdx;
                        }
                        DEBUG_PRINT(("tblSize: %d\n", tblTracked.size()));
                        // Go through the LUT to process the feature points
                        for (int i=0; i<tblTracked.size(); i++)
                        {
                            if(tblTracked[i].tracked == false)  // untracked new features
                            {
//                                DEBUG_PRINT(("New point idx: %d\n", i));
                                cv::Rect roi = cv::Rect(0, i, matDescriptors.cols, 1);


                                matDescriptorPool.push_back(matDescriptors(roi));

                                circle(frame, vecKeypoints[i].pt, 7, Scalar(0, 255, 0), 3);
                                putText(frame, boost::lexical_cast<string>(matDescriptorPool.rows), vecKeypoints[i].pt+fontShift_feature,
                                        fontFace, fontScale_feature, Scalar(0, 255, 0), 1, 8);
                            }
                            else
                            {
                                circle(frame, vecKeypoints[i].pt, 3, Scalar(0, 0, 255), 3);
                                putText(frame, boost::lexical_cast<string>(tblTracked[i].trackedIDinPool), vecKeypoints[i].pt+fontShift_feature,
                                        fontFace, fontScale_feature, Scalar(0, 0, 255), 1, 8);




//                                line(frame, vecKeypoints[i].pt, tblTracked[i].trackedIDinPool)
                            }
                        }


                    }
                    else
                    {
                        printf("!!!No match\n");
//                        matDescriptorPool.push_back(matDescriptors);
//                        for(int i = 0; i<vecKeypoints.size(); i++)
//                        {
//                            circle(frame, vecKeypoints[i].pt, 7, Scalar(0, 255, 0), 3);
//                            putText(frame, boost::lexical_cast<string>(i), vecKeypoints[i].pt, fontFace, fontScale,
//                                    Scalar(0, 255, 0), 1, 8);
//                        }
                    }
                    DEBUG_PRINT(("Pool length: %d\n", matDescriptorPool.rows));
                }


                //                    circle(frame, vecKeypoints[i].pt, cvRound(vecKeypoints[i].response/1000.0), Scalar(0, 0, 255), 3);


            }
            printf("-----\n");

        }
        else if(current_state == STATE_READY)
        {
            //            printf("STATE_READY\n");
        }


        Size textSize = getTextSize(text_state, fontFace, fontScale, thickness, &baseline);
        baseline += thickness;
        // center the text
        Point textOrg(0, (frame.rows - textSize.height/2));
        // draw the box
        //        rectangle(frame, textOrg + Point(0, baseline),
        //                  textOrg + Point(textSize.width, -textSize.height),
        //                  Scalar(0,0,255));
        // ... and the baseline first
        //        line(frame, textOrg + Point(0, thickness),
        //             textOrg + Point(textSize.width, thickness),
        //             Scalar(0, 0, 255));
        // then put the text itself
        putText(frame, text_state, textOrg, fontFace, fontScale,
                color_state, thickness, 8);
        imshow("Feature", frame);




        img_for_pub.encoding = "bgr8";
        img_for_pub.image = frame;
        img_for_pub.header.stamp = ros::Time::now();
        pub_frame.publish(img_for_pub);

        keyIn = waitKey(30);
        switch(keyIn)
        {
        case 27:
            current_state = STATE_EXIT;
            printf("\nSTATE_EXIT\n");
            break;
        case 't':
            if(current_state == STATE_TRACKING)
            {
                current_state = STATE_READY;
                text_state = "Tracking: Off";
                color_state.val[0] = 255;
                color_state.val[1] = 0;
                printf("\nSTATE_READY\n");
            }
            else if(current_state == STATE_READY)
            {
                current_state = STATE_TRACKING;
                text_state = "Tracking: On";
                color_state.val[0] = 0;
                color_state.val[1] = 255;
                printf("\nSTATE_TRACKING\n");
            }
            break;
        case 's':
            String fileName = fileName_prefix + boost::lexical_cast<string>(fileName_numbering) + fileName_suffix;
            imwrite(fileName, frame_gray);
            fileName_numbering++;
            printf("%s is saved\n", fileName.c_str());
        }



        //r.sleep();


    }

    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
