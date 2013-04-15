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

        buildFrameFromImage(img_disk);

        obj_width *= 5.0;
        obj_height *= 5.0;

        //-- set up the scale and centroid
        // scale unit: mm/pix
        scale = obj_width / jlUtilities::compute_magnitude(vec_corners[0],
                                                           vec_corners[1]);
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