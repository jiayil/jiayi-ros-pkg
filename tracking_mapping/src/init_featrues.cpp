/* File:    init_featrues.cpp
   Author:  Jiayi Liu
   Date:    Mar 29, 2013
   Description:
            Initialize a list of features for object detection

  */

#include <cstdio>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include "jlUtilities/opencv_helper.h"


using namespace cv;


int main(int argc, char** argv)
{
    bool flag_use_image = true;
    if( argc != 2 )
      {
        std::cout<< "Usage: ./init_features num" << std::endl;
        std::cout<< "num: 0 - image"
                 << "     1 - video" << std::endl;
        return -1;
    }
    else
    {
        std::string val = argv[1];
        if(val == "0")
        {

        }
        else if(val == "1")
        {
            flag_use_image = false;
        }
        else
        {
            std::cout<< "num error" << std::endl;
        }
    }




    if(flag_use_image == true)
    {
        namedWindow("Keypoints", WINDOW_NORMAL);
        Mat mat_image = imread( "../data/cam_spf2.jpg", CV_LOAD_IMAGE_GRAYSCALE );
        int num_vecKeypoints;
        int num_trackingPoints = 50;
        Mat mat_descriptors;

        //-- Step 1: Detect the keypoints using Detector
        // int minHessian = 400;





        OrbFeatureDetector detector;
        OrbDescriptorExtractor extractor;

        std::vector<KeyPoint> vec_keypoints, vec_goodKeypoints;

        detector.detect( mat_image, vec_keypoints );
        num_vecKeypoints = vec_keypoints.size();

        std::sort(vec_keypoints.begin(), vec_keypoints.end(),
                  jlUtilities::sort_feature_response);

        if(num_vecKeypoints > num_trackingPoints)
        {
            num_vecKeypoints = num_trackingPoints;
            vec_keypoints.erase(vec_keypoints.begin() + num_vecKeypoints,
                               vec_keypoints.end());
        }


        extractor.compute( mat_image, vec_keypoints, mat_descriptors );


        // write mat to file
        std::string fileName = "mat_descriptors.yml";
        FileStorage fs(fileName, FileStorage::WRITE);
        fs << "descriptors" << mat_descriptors;
        fs.release();
        std::cout<< fileName << " is generated." << std::endl;

//        Mat copy;
//        FileStorage fs2("mat_descriptors.yml", FileStorage::READ);
//        fs2["descriptors"] >> copy;
//        fs2.release();

//        FileStorage fs3("test.yml", FileStorage::WRITE);
//        fs3 << "descriptors" << copy;
//        fs3.release();


        //////////////////////////////////////////////////////////
//        std::vector<cv::Point3f> vec_pois;
//        vec_pois.push_back(Point3f(0, 0, 0));
//        vec_pois.push_back(Point3f(1.1, 0.1, 0));
//        vec_pois.push_back(Point3f(0.3, 2.1, 0));
//        vec_pois.push_back(Point3f(7.3, 2, 0));
//        vec_pois.push_back(Point3f(1.3, 4.1, 0));

//        FileStorage fs3("POIs.yml", FileStorage::WRITE);
//        fs3 << "POIs" << vec_pois;
//        fs3.release();

        //////////////////////////////////////////////////////////

        //-- Draw keypoints
        Mat mat_kpImage;

        drawKeypoints( mat_image, vec_keypoints, mat_kpImage,
                       Scalar::all(-1), DrawMatchesFlags::DEFAULT );

        for (int i=0; i<num_trackingPoints; i++)	{
            cv::circle(mat_kpImage,
                vec_keypoints[i].pt,	// center
                3,							// radius
                cv::Scalar(0,0,255),		// color
                -1);						// negative thickness=filled

            char szLabel[50];
            sprintf(szLabel, "%d", i);
            putText (mat_kpImage, szLabel, vec_keypoints[i].pt,
                cv::FONT_HERSHEY_PLAIN, // font face
                1.0,					// font scale
                cv::Scalar(255,0,0),	// font color
                1);						// thickness
        }


        //-- Show detected (drawn) keypoints
        imshow("Keypoints", mat_kpImage );

        waitKey(0);

    }
    else // video input: tracking features
    {
        VideoCapture cap;

        cap.open(1);
        if(!cap.isOpened())  // check if we succeeded
            return -1;
        cap.set(CV_CAP_PROP_FRAME_WIDTH, 800);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, 600);


        namedWindow("Keypoints", WINDOW_NORMAL);
        Mat mat_image;
        int num_vecKeypoints;
        int num_trackingPoints = 50;
        Mat mat_descriptors;

        char keyInput;

        //-- Step 1: Detect the keypoints using Detector
        // int minHessian = 400;





        OrbFeatureDetector detector;
        FREAK extractor;

        while(1)
        {
            cap >> mat_image;

            std::vector<KeyPoint> vec_keypoints, vec_goodKeypoints;

            detector.detect( mat_image, vec_keypoints );
            num_vecKeypoints = vec_keypoints.size();

            std::sort(vec_keypoints.begin(), vec_keypoints.end(),
                      jlUtilities::sort_feature_response);

            if(num_vecKeypoints > num_trackingPoints)
            {
                num_vecKeypoints = num_trackingPoints;
                vec_keypoints.erase(vec_keypoints.begin() + num_vecKeypoints,
                                   vec_keypoints.end());
            }


            extractor.compute( mat_image, vec_keypoints, mat_descriptors );


            // write mat to file
            std::string fileName = "mat_descriptors.yml";
            FileStorage fs(fileName, FileStorage::WRITE);
            fs << "descriptors" << mat_descriptors;
            fs.release();
            std::cout<< fileName << " is generated." << std::endl;

    //        Mat copy;
    //        FileStorage fs2("mat_descriptors.yml", FileStorage::READ);
    //        fs2["descriptors"] >> copy;
    //        fs2.release();

    //        FileStorage fs3("test.yml", FileStorage::WRITE);
    //        fs3 << "descriptors" << copy;
    //        fs3.release();


            //////////////////////////////////////////////////////////
    //        std::vector<cv::Point3f> vec_pois;
    //        vec_pois.push_back(Point3f(0, 0, 0));
    //        vec_pois.push_back(Point3f(1.1, 0.1, 0));
    //        vec_pois.push_back(Point3f(0.3, 2.1, 0));
    //        vec_pois.push_back(Point3f(7.3, 2, 0));
    //        vec_pois.push_back(Point3f(1.3, 4.1, 0));

    //        FileStorage fs3("POIs.yml", FileStorage::WRITE);
    //        fs3 << "POIs" << vec_pois;
    //        fs3.release();

            //////////////////////////////////////////////////////////

            //-- Draw keypoints
            Mat mat_kpImage;

            drawKeypoints( mat_image, vec_keypoints, mat_kpImage,
                           Scalar::all(-1), DrawMatchesFlags::DEFAULT );

            for (int i=0; i<num_trackingPoints; i++)	{
                cv::circle(mat_kpImage,
                    vec_keypoints[i].pt,	// center
                    3,							// radius
                    cv::Scalar(0,0,255),		// color
                    -1);						// negative thickness=filled

                char szLabel[50];
                sprintf(szLabel, "%d", i);
                putText (mat_kpImage, szLabel, vec_keypoints[i].pt,
                    cv::FONT_HERSHEY_PLAIN, // font face
                    1.0,					// font scale
                    cv::Scalar(255,0,0),	// font color
                    1);						// thickness
            }


            //-- Show detected (drawn) keypoints
            imshow("Keypoints", mat_kpImage );

            waitKey(30);
        }


    }


    return 0;
}
