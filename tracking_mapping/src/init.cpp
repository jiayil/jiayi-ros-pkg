
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
#include "tracking_mapping/cv/dataset_ucsb.h"

#define PI 3.14159265358979323846

using namespace cv;

Mat mat_canvas;
std::vector<cv::Point2f> vec_corners;
Mat mat_homography;
Mat mat_intrinsics, mat_distCoeffs;

bool flag_is_written = false;

void preprocessFrame(Mat& frame, Mat& matIntrin, Mat& matDist)
{
printf("beforebefore\n");

//    IplImage* mx = cvCreateImage( Size(frame.cols, frame.rows), IPL_DEPTH_32F, 1 );
//    IplImage* my = cvCreateImage( Size(frame.cols, frame.rows), IPL_DEPTH_32F, 1 );

    Mat mx(frame.rows, frame.cols, CV_32FC1);
    Mat my(frame.rows, frame.cols, CV_32FC1);

    printf("intrin: %dx%d\n", matIntrin.cols, matIntrin.rows);
    CvMat cvmatMx = mx;
    CvMat cvmatMy = my;

    CvMat cvmatIntrin = matIntrin;
    CvMat cvmatDist = matDist;
    printf("before0\n");
    jlUtilities::my_cvInitUndistortMap( &cvmatIntrin, &cvmatDist, &cvmatMx, &cvmatMy );
    printf("before\n");
    CvMat cvmatFrame = frame;
    cvRemap( &cvmatFrame, &cvmatFrame, &cvmatMx, &cvmatMy); // undistort image
    printf("after\n");

}

void outputParam()
{
    float obj_width = 216; // 220mm
    float obj_height = 279; // 240mm

    // my Logitech cam intrinsics
    double K_[3][3] =
    { {881.68473, 0, 398.14820},
    {0,  884.03796, 289.57924},
    {0,   0,   1} };



    mat_intrinsics = cv::Mat(3, 3, CV_64F, K_).clone();



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

    // my Logitech cam distortion coeffs
    double dC[5] = {0.09026, -0.13131, 0.00076, 0.00090, 0};

    mat_distCoeffs = cv::Mat(5, 1, CV_64F, dC).clone();




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
        jlUtilities::draw_pointCoord(mat_canvas, p);

    }

    if(vec_corners.size() == 4 && flag_is_written == false)
    {
        outputParam();

        flag_is_written = true;
    }




}

void useDataset()
{
    float obj_width = 203.2; //216; // 220mm
    float obj_height = 139.7; //279; // 240mm


    // UCSB dataset
    double K_[3][3] =
    { {869.57, 0, 299.748},
      {0,  867.528, 237.284	},
      {0,   0,   1} };

    Mat mat_intrinsics = cv::Mat(3, 3, CV_64F, K_).clone();
    printf("intrinsics is set\n");


    // extrinsics
    Mat mat_extrinsics = Mat::zeros(3, 4, CV_64F);
    mat_extrinsics.at<double>(2, 3) = 595;  // 400mm
    printf("extrinsics is set\n");


    tf::Matrix3x3 rotMatrix;
    rotMatrix.setRPY(PI, 0, PI/2.0);
    rotMatrix = rotMatrix.inverse();

    for(int i; i<3; i++)
    {
        mat_extrinsics.at<double>(i, 0) = rotMatrix.getRow(i).getX();
        mat_extrinsics.at<double>(i, 1) = rotMatrix.getRow(i).getY();
        mat_extrinsics.at<double>(i, 2) = rotMatrix.getRow(i).getZ();

    }


    // UCSB dataset
    double dC[5] = {-0.0225415, -0.259618, 0.00320736, -0.000551689, 0};
    Mat mat_distCoeffs = cv::Mat(5, 1, CV_64F, dC).clone();
    printf("distCoeff is set\n");

    // UCSB dataset homography to warp the image to the canonical ref frame
    double H_bu_zm[3][3] =
    { {0.770604, 0.045646, -36.1442},
      {0.00590527, 0.81837, -38.1734},
      {2.67426e-05, 0.000254044, 1}};


    double H_bu_ls[3][3] =
    { {1.11578, 0.165125, -122.249},
      {-0.030038, 1.56984, -177.268},
      {-9.60671e-05, 0.00107615, 1}};

    mat_homography = cv::Mat(3, 3, CV_64F, H_bu_zm).clone();
    printf("Homo is set\n");



    preprocessFrame(mat_canvas, mat_intrinsics, mat_distCoeffs);


//    std::vector<cv::Point2f> corners_warped;
//    Mat H_inv = mat_homography.inv();
//    perspectiveTransform(vec_corners, corners_warped, H_inv);
//    jlUtilities::draw_pointCoord(mat_canvas, corners_warped);

    Mat matTemp;
    warpPerspective(mat_canvas, matTemp, mat_homography, Size(dst_w, dst_h));

//    printf("cannonical: %dx%d\n", dst_w, dst_h);
//    mat_canvas.setTo(0);
//    Point2f pTrans;
//    pTrans.x = (mat_intrinsics.at<double>(0,2) - matTemp.cols/2);
//    pTrans.y = (mat_intrinsics.at<double>(1,2) - matTemp.rows/2);
//    for(int i=0; i<4;i++)
//    {
//        Point2f p;
//        p.x = textureROI_p[i].x;
//        p.y = textureROI_p[i].y;
//        p += pTrans;
//        std::cout<<p<<std::endl;
//        vec_corners.push_back(p);
//    }
//    matTemp.copyTo(mat_canvas(cv::Rect(pTrans.x, pTrans.y, dst_w, dst_h)));
//    printf("%dx%d\n", matTemp.rows, matTemp.cols);

    mat_canvas = matTemp;
    for(int i=0; i<4;i++)
    {
        Point2f p;
        p.x = textureROI_p[i].x;
        p.y = textureROI_p[i].y;
        std::cout<<p<<std::endl;
        vec_corners.push_back(p);
    }
    imwrite( "book_canonical.jpg", mat_canvas );
    jlUtilities::draw_pointCoord(mat_canvas, vec_corners);


    // write vec to file
    std::string fileName = "data/book.yml";
    FileStorage fs(fileName, FileStorage::WRITE);
    fs << "objWidth" << obj_width;
    fs << "objHeight" << obj_height;
    fs << "corners" << vec_corners;  // corner coords in canonical image
    fs << "intrinsics" << mat_intrinsics;
    fs << "extrinsics" << mat_extrinsics;
    fs << "distCoeffs" << mat_distCoeffs;
    fs << "scale"      << 1.0f;
    fs.release();
    std::cout<< fileName << " is generated." << std::endl;

}



int main(int argc, char** argv)
{
    int flag_use_image = 0;
    if( argc != 2 )
      {
        std::cout<< "Usage: ./init num" << std::endl;
        std::cout<< "num: 0 - image" << std::endl
                 << "     1 - video" << std::endl
                 << "     2 - dataset" << std::endl;
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
        else if(val == "2")
        {
            flag_use_image = 2;
        }
        else
        {
            std::cout<< "num error" << std::endl;
        }
    }

    std::string winName = "Image";
    namedWindow(winName, WINDOW_NORMAL);
    mat_canvas = imread( "data/book.jpg");

    if(mat_canvas.data == NULL)
    {
        std::cout<< "Image is not opened." << std::endl;
        return -1;
    }


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
    //-- use dataset
    else if(flag_use_image == 2)
    {



        useDataset();





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
