/* File:    Initializer.cpp
   Author:  Jiayi Liu
   Date:    Mar 25, 2013
   Description:


  */

#include <cstdio>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include "tracking_mapping/tm/Initializer.h"


using namespace cv;

std::vector<Point> vec_init_corners_temp;

////callback function
//void mouseEvent(int evt, int x, int y, int flags, void* param)
//{
//    if(evt==CV_EVENT_LBUTTONDOWN)
//    {
//        printf("Mouse clicked on: %d %d\n",x,y);

//        Point p = Point(x, y);
//        vec_init_corners_temp.push_back(p);
//    }
//}

Initializer::Initializer()
{
    if(pattern.current_state == pattern.PATTERN_STATE_SUCCESS)
    {
        current_state = INIT_STATE_SUCCESS;
        printf("Initializer is constructed.\n");
    }
    else
    {
        current_state = INIT_STATE_FAILED;
    }
}

/*
 * Match features between the current image and the training image,
 * and compute the pose of the current camera wrt the current image,
 * assuming the center of the current image is the origin of the world.
 */
// Add the result of the motion model later, reducing the feature extraction cost.
bool Initializer::process(cv::Mat &image)
{
    //-- Match features
    matches.clear();

    //-- Fill up the frame
    current_frame.buildFrameFromImage(image);
    printf("Initializer: img counter: %d\n", current_frame.frame_counter);

    //-- Match features
    matchFeatures();

    if(matches.size() < 4)
    {

        current_state = INIT_STATE_SUCCESS;
        return false;
    }
//printf("Match size: %d.\n", matches.size());
    //-- Compute homography
    Mat H;
    bool find_H = computeHomography(H);
    if(!find_H)
    {
        current_state = INIT_STATE_SUCCESS;
        return false;
    }

    //-- Warp pattern corners into the current image
    cv::perspectiveTransform(pattern.vec_corners,
                             pattern.vec_corners_currentImg,
                             H);
    for(int i; i<4; i++)
        std::cout<< pattern.vec_corners_currentImg[i].x << " " << pattern.vec_corners_currentImg[i].y << std::endl;
    std::cout<< std::endl;
    //-- Compute the camera pose
    bool find_pose = computePoseCorners();
    if(!find_pose)
    {
        current_state = INIT_STATE_SUCCESS;
        return false;
    }

    current_state = INIT_STATE_POSE_COMPUTED;
    return true;

}

void Initializer::matchFeatures()
{
    // matcher is put here, can't be initialized in Initializer's definition.
    cv::BFMatcher matcher(cv::NORM_HAMMING);
//    FlannBasedMatcher matcher;
    std::vector<std::vector< DMatch > > knnMatches;

    matcher.knnMatch(current_frame.mat_descriptors,
                     pattern.mat_descriptors,
                     knnMatches, 2 );

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
    }

//    printf("Initializer: Matches length: %d\n", matches.size());

//    for(int i=0;i<matches.size();i++)
//        printf("q: %d, t: %d\n", matches[i].queryIdx, matches[i].trainIdx);

}
bool Initializer::computeHomography(Mat &H)
{
    // Prepare data for cv::findHomography
    std::vector<cv::Point2f> srcPoints(matches.size());
    std::vector<cv::Point2f> dstPoints(matches.size());

    for (size_t i = 0; i < matches.size(); i++)
    {
        srcPoints[i] = pattern.vec_keypoints[matches[i].trainIdx].pt;
        dstPoints[i] = current_frame.vec_keypoints[matches[i].queryIdx].pt;
    }


    // Find homography matrix and get inliers mask
    std::vector<unsigned char> inliersMask(srcPoints.size());
    H = cv::findHomography(srcPoints,
                           dstPoints,
                           CV_FM_RANSAC,
                           3,
                           inliersMask);

    std::vector<cv::DMatch> inliers;
    for (size_t i=0; i<inliersMask.size(); i++)
    {
        if (inliersMask[i])
            inliers.push_back(matches[i]);
    }

    matches.swap(inliers);
    printf("inliers size: %d\n", matches.size());
    return matches.size() >= 4;
}

bool Initializer::computePoseCorners()
{

    cv::Mat Rvec;
    cv::Mat_<float> Tvec;
    cv::Mat raux, taux;

    cv::solvePnP(pattern.vec_obj_corners3d,
                       pattern.vec_corners_currentImg,
                       pattern.camera.mat_intrinsics,
                       pattern.camera.mat_distCoeffs,
                       raux,
                       taux);

    //-- Matrix conversion
    raux.convertTo(Rvec,CV_32F);
    taux.convertTo(Tvec ,CV_32F);

    cv::Mat_<float> rotMat(3,3);
    cv::Rodrigues(Rvec, rotMat);

    if(rotMat(0,0) == 1 && rotMat(1,1) == 1 && rotMat(2,2) == 1 )
    {
        printf("Identity\n");
        return false;
    }

    Matx34d M;
    // Copy to transformation matrix
    for (int row=0; row<3; row++)
    {
      for (int col=0; col<3; col++)
      {
          // Copy rotation component
          M(row, col) = rotMat(row, col);
      }
      M(row, 3) = Tvec(row)/1000.0; // Copy translation component
    }
    pattern.camera.mat_transform = Mat(M);
    std::cout << pattern.camera.mat_transform << std::endl;
    return true;
}

void Initializer::computePose()
{
//    Matx34d M;


//    getCameraMat(initializer.mat_intrinsics,
//                 initializer.vec_current_keypoints,
//                 initializer.vec_keypoints,
//                 M,
//                 initializer.matches);

//    camM = Mat(M);

//    std::cout << camM << std::endl;
}
