/* File:    Pattern.cpp
   Author:  Jiayi Liu
   Date:    Apr 14, 2013
   Description:


  */
#include <cstdio>

#include "tracking_mapping/cv/Pattern.h"
#include "jlUtilities/opencv_helper.h"

Pattern::Pattern()
{
    //-- Read in the file of parameters
    std::string file_name = "data/book.yml";
    FileStorage fs;
    fs.open(file_name, FileStorage::READ);
    if(fs.isOpened())
    {
        printf("Initializer: %s is opened for reading.\n", file_name.c_str());
        fs["objWidth"]      >> obj_width;
        fs["objHeight"]     >> obj_height;
        fs["intrinsics"]    >> camera.mat_intrinsics;
        fs["extrinsics"]    >> camera.mat_extrinsics;
        fs["corners"]       >> vec_corners;
        fs["distCoeffs"]    >> camera.mat_distCoeffs;

        fs.release();

        printf("Values are\n");
        std::cout << "objWidth: " << obj_width << std::endl
                  << "objHeight: " << obj_height << std::endl
                  << "intrinsics:\n" << camera.mat_intrinsics << std::endl
                  << "extrinsics:\n" << camera.mat_extrinsics << std::endl
                  << "corners: " << vec_corners << std::endl
                  << "distCoeffs:\n" << camera.mat_distCoeffs << std::endl;

        printf("Pattern is constructed.\n");
        current_state = PATTERN_STATE_SUCCESS;
    }
    else
    {
        printf("Pattern: Error! Failed to open parameter file: %s\n", file_name.c_str());
        current_state = PATTERN_STATE_FAILED;
    }

    //-- read in the training image
    file_name = "data/book.jpg";
    Mat img_disk = imread( file_name);

    if(img_disk.data != NULL)
    {
        printf("Pattern: %s is read in for feature initialization.\n", file_name.c_str());

        buildPatternFromImage(img_disk);

        // arbitrary scaling for better trajectory visualization
//        obj_width *= 5.0;
//        obj_height *= 5.0;

        //-- set up the scale and centroid
        // scale unit: mm/pix
        scale = obj_width / (vec_corners[2].x - vec_corners[0].x);
        obj_img_center = jlUtilities::compute_centroid(vec_corners[0],
                                                     vec_corners[2]);



        // ros; center
        vec_obj_corners3d.push_back(Point3f(obj_height/2.0f, obj_width/2.0f, 0));
        vec_obj_corners3d.push_back(Point3f(obj_height/2.0f, -obj_width/2.0f, 0));
        vec_obj_corners3d.push_back(Point3f(-obj_height/2.0f, -obj_width/2.0f, 0));
        vec_obj_corners3d.push_back(Point3f(-obj_height/2.0f, obj_width/2.0f, 0));

        //        vec_obj_corners3d.push_back(Point3f(-obj_width/2.0f, obj_height/2.0f, 0));
        //        vec_obj_corners3d.push_back(Point3f(obj_width/2.0f, obj_height/2.0f, 0));
        //        vec_obj_corners3d.push_back(Point3f(obj_width/2.0f, -obj_height/2.0f, 0));
        //        vec_obj_corners3d.push_back(Point3f(-obj_width/2.0f, -obj_height/2.0f, 0));

//        // obj coords; center
//        vec_obj_corners3d.push_back(Point3f(-obj_width/2.0f, -obj_height/2.0f, 0));
//        vec_obj_corners3d.push_back(Point3f(obj_width/2.0f, -obj_height/2.0f, 0));
//        vec_obj_corners3d.push_back(Point3f(obj_width/2.0f, obj_height/2.0f, 0));
//        vec_obj_corners3d.push_back(Point3f(-obj_width/2.0f, obj_height/2.0f, 0));

        printf("Image size: %dx%d. Scale: %f mm/pix\n", img_width, img_height, scale);
        printf("Descriptors Dim: %d; Col: %d; Row: %d\n",
               mat_descriptors.dims,
               mat_descriptors.cols,
               mat_descriptors.rows);
    }
    else
    {
        printf("Pattern: Error! Failed to open image: %s\n", file_name.c_str());
        current_state = PATTERN_STATE_FAILED;
    }

}

void Pattern::buildPatternFromImage(Mat &img)
{
    mat_image = img.clone();
    cvtColor( mat_image, mat_image_gray, CV_BGR2GRAY );
    img_width = mat_image.cols;
    img_height = mat_image.rows;
    frame_counter++;

    vec_matchedFeaturePoints.clear();

    //-- CV

    cv::Mat imageROI;
    imageROI = mat_image_gray(cv::Rect(vec_corners[0], vec_corners[2]));


    detector.detect( imageROI, vec_keypoints );
    for(size_t i=0; i<vec_keypoints.size(); i++)
    {
        vec_keypoints[i].pt += vec_corners[0];
    }


    extractor.compute( mat_image_gray, vec_keypoints, mat_descriptors );

    printf("Keypoints size: %d.\n", vec_keypoints.size());

}
