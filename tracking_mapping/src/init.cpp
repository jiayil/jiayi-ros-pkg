
/* File:    init.cpp
   Author:  Jiayi Liu
   Date:    Apr 7, 2013
   Description:
            Initialize for tracking

  */

#include <cstdio>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <tf/transform_listener.h>

#include "jlUtilities/opencv_helper.h"

#define PI 3.14159265358979323846

using namespace cv;

Mat mat_canvas;
std::vector<cv::Point> vec_corners;

bool flag_is_written = false;

void outputParam()
{
    float obj_width = 216; // 220mm
    float obj_height = 279; // 240mm

    // intrinsics
    double K_[3][3] =
    { {881.68473, 0, 398.14820},
    {0,  884.03796, 289.57924},
    {0,   0,   1} };

    Mat mat_intrinsics = cv::Mat(3, 3, CV_64F, K_).clone();



    // extrinsics
    Mat mat_extrinsics = Mat::zeros(3, 4, CV_64F);
    mat_extrinsics.at<double>(2, 3) = 595;  // 400mm



    tf::Matrix3x3 rotMatrix;
    rotMatrix.setRPY(PI, 0, PI/2.0);
    rotMatrix = rotMatrix.inverse();

    for(int i; i<3; i++)
    {
        mat_extrinsics.at<double>(i, 0) = rotMatrix.getRow(i).getX();
        mat_extrinsics.at<double>(i, 1) = rotMatrix.getRow(i).getY();
        mat_extrinsics.at<double>(i, 2) = rotMatrix.getRow(i).getZ();

    }

    // distortion coeffs
    double dC[5] = {0.09026, -0.13131, 0.00076, 0.00090, 0};
    Mat mat_distCoeffs = cv::Mat(5, 1, CV_64F, dC).clone();


    // write vec to file
    std::string fileName = "data/book.yml";
    FileStorage fs(fileName, FileStorage::WRITE);
    fs << "objWidth" << obj_width;
    fs << "objHeight" << obj_height;
    fs << "corners" << vec_corners;
    fs << "intrinsics" << mat_intrinsics;
    fs << "extrinsics" << mat_extrinsics;
    fs << "distCoeffs" << mat_distCoeffs;
    fs.release();
    std::cout<< fileName << " is generated." << std::endl;
}

//callback function
void mouseEvent(int evt, int x, int y, int flags, void* param)
{
    if(evt==CV_EVENT_LBUTTONDOWN)
    {
        printf("Mouse clicked on: %d %d\n",x,y);

        Point p = Point(x, y);
        vec_corners.push_back(p);
        cv::circle(mat_canvas,
                   p,	// center
                   3,							// radius
            cv::Scalar(0,0,255),		// color
            -1);
        char szLabel[50];
        sprintf(szLabel, "(%d, %d)", x, y);
        putText (mat_canvas, szLabel, p,
            cv::FONT_HERSHEY_PLAIN, // font face
            1.0,					// font scale
            cv::Scalar(255,0,0),	// font color
            1);						// thickness
    }

    if(vec_corners.size() == 4 && flag_is_written == false)
    {
        outputParam();

        flag_is_written = true;
    }




}

int main(int argc, char** argv)
{
    int flag_use_image = 0;
    if( argc != 2 )
      {
        std::cout<< "Usage: ./init num" << std::endl;
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
            flag_use_image = 1;
        }
        else
        {
            std::cout<< "num error" << std::endl;
        }
    }

    std::string winName = "Image";
    namedWindow(winName, WINDOW_NORMAL);
    mat_canvas = imread( "data/book.jpg");


    if(flag_use_image == 0)
    {



        setMouseCallback(winName, mouseEvent);




//        // write mat to file
//        std::string fileName = "mat_descriptors.yml";
//        FileStorage fs(fileName, FileStorage::WRITE);
//        fs << "descriptors" << mat_descriptors;
//        fs.release();
//        std::cout<< fileName << " is generated." << std::endl;

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




        while(1)
        {
            imshow(winName, mat_canvas );

            waitKey(30);
        }

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
