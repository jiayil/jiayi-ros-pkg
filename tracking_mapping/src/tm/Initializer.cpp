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

Initializer::Initializer()
{
    // Read in the file of descriptors

    FileStorage fs;
    fs.open("bin/mat_descriptors.yml", FileStorage::READ);
    if(fs.isOpened())
    {
        fs["descriptors"] >> mat_poi_descriptors;
        cv::Rect roi = cv::Rect(0, 0, mat_poi_descriptors.cols, 5);
        mat_roi = mat_poi_descriptors(roi);
        fs.release();
    }
    else
    {
        printf("Fail to open descriptor file.\n");

    }


    fs.open("bin/POIs.yml", FileStorage::READ);
    if(fs.isOpened())
    {
        fs["POIs"] >> vec_objCords;
        fs.release();
    }
    else
    {
        printf("Fail to open POI file.\n");
    }

    printf("Initializer constructed.\n");
}

Initializer::Initializer(std::string &name)
{








    printf("Initializer constructed.\n");
}

// Add the result of the motion model later, reducing the feature extraction cost.
bool Initializer::process(cv::Mat &mat_image)
{

    matches.clear();

    OrbFeatureDetector detector;
    OrbDescriptorExtractor extractor;

    Mat mat_current_descriptors;


    detector.detect( mat_image, vec_current_keypoints );


    extractor.compute( mat_image, vec_current_keypoints, mat_current_descriptors );




    std::cout<< "Current descriptors: " << mat_current_descriptors.rows << std::endl
             << "POI descriptors: " << mat_roi.rows << std::endl;

    BFMatcher matcher(NORM_HAMMING);

    //FlannBasedMatcher matcher;

    std::vector<std::vector< DMatch > > knnMatches;

    matcher.knnMatch(mat_current_descriptors, mat_roi, knnMatches, 2 );
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




    std::cout<< matches.size() << std::endl;

    for(int i=0;i<matches.size();i++)
        printf("q: %d, t: %d\n", matches[i].queryIdx, matches[i].trainIdx);

//    if(matches.size() == 5)
//    {
//        for(int i=0;i<5;i++)
//        {

//        }
//    }

    return false;
}
