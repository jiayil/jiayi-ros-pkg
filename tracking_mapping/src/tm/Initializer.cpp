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
    counter = 0;
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
//    std::cout << "Initializer: img counter: " << current_frame.frame_counter << std::endl;

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

    //-- (canceled) Warp pattern corners into the current image
//    cv::perspectiveTransform(pattern.vec_corners,
//                             pattern.vec_corners_currentImg,
//                             H);
//    for(int i; i<4; i++)
//        std::cout<< pattern.vec_corners_currentImg[i].x << " " << pattern.vec_corners_currentImg[i].y << std::endl;
//    std::cout<< std::endl;

    //-- (instead) Warp all the matched points
    cv::perspectiveTransform(pattern.vec_matchedFeaturePoints,
                             pattern.vec_matchedFeaturePoints_currentImg,
                             H);
    cv::perspectiveTransform(pattern.vec_corners,
                             pattern.vec_corners_currentImg,
                             H);

    //-- Compute 3D locations of Pattern's matched features in the current image
    computeLocation3D(pattern.vec_matchedFeaturePoints,
                      pattern.vec_obj_matchedFeaturePoints3d,
//                      Point2f(400, 300),
                      pattern.obj_img_center,
                      pattern.scale);

//    printf("center x: %d, y: %d\n",
//           pattern.obj_img_center.x,
//           pattern.obj_img_center.y);



    //-- Compute the camera pose
    bool find_pose = computePoseCorners(pattern.vec_obj_matchedFeaturePoints3d,
                                        pattern.vec_matchedFeaturePoints_currentImg);
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
    pattern.vec_matchedFeaturePoints.clear();

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

            pattern.vec_matchedFeaturePoints.push_back(
                        pattern.vec_keypoints[bestMatch.trainIdx].pt);
            current_frame.vec_matchedFeaturePoints.push_back(
                        current_frame.vec_keypoints[bestMatch.queryIdx].pt);
        }
    }

//    printf("Initializer: Matches length: %d\n", matches.size());

//    for(int i=0;i<matches.size();i++)
//        printf("q: %d, t: %d\n", matches[i].queryIdx, matches[i].trainIdx);

}
bool Initializer::computeHomography(Mat &H)
{
    // Prepare data for cv::findHomography
    std::vector<cv::Point2f>& srcPoints = pattern.vec_matchedFeaturePoints;
    std::vector<cv::Point2f>& dstPoints = current_frame.vec_matchedFeaturePoints;

    // The following has been done in the matching stage
//    for (size_t i = 0; i < matches.size(); i++)
//    {
//        srcPoints[i] = pattern.vec_keypoints[matches[i].trainIdx].pt;
//        dstPoints[i] = current_frame.vec_keypoints[matches[i].queryIdx].pt;
//    }


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
//    printf("inliers size: %d\n", matches.size());
    return matches.size() >= 4;
}

bool Initializer::computePoseCorners(std::vector<cv::Point3f> &pts3D,
                                     std::vector<cv::Point2f> &pts2D)
{

    cv::Mat Rvec;
    cv::Mat_<float> Tvec;
    cv::Mat raux, taux;

//    std::cout<< counter << std::endl;
    counter++;
    cv::solvePnPRansac(pts3D,
                 pts2D,
                 pattern.camera.mat_intrinsics,
                 pattern.camera.mat_distCoeffs,
                 raux,
                 taux);
//    std::cout<< "finish pnp" << std::endl;
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

//    Matx44d M;
    Matx34d M;
    // Copy to transformation matrix
    for (int row=0; row<3; row++)
    {
      for (int col=0; col<3; col++)
      {
          // Copy rotation component
          M(row, col) = rotMat(row, col);
      }
      M(row, 3) = Tvec(row); // Copy translation component
    }
//    M(3, 3) = 1;

    pattern.camera.mat_transform = Mat(M);

//    std::cout << pattern.camera.mat_transform << std::endl;
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

void Initializer::computeLocation3D(std::vector<cv::Point2f> &pts2D,
                       std::vector<cv::Point3f> &pts3D,
                       cv::Point2f centerPoint,
                       float scale)
{
    pts3D.clear();
    // scale unit: mm/pix

    for(size_t i=0;i<pts2D.size();i++)
    {
        cv::Point3f p;
        p.x = (centerPoint.y - pts2D[i].y) * scale;
        p.y = (centerPoint.x - pts2D[i].x) * scale;
        p.z = 0;
        pts3D.push_back(p);
    }
}
