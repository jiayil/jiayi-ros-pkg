/* File:    opencv_helper.cpp
   Author:  Jiayi Liu
   Date:    Mar 17, 2013
   Description:


  */

#include "jlUtilities/opencv_helper.h"
#include "jlUtilities/jlUtilities.h"

namespace jlUtilities
{
bool sort_feature_response(const KeyPoint &first, const KeyPoint &second)
{
    return first.response > second.response;
}

double compute_magnitude(Point a, Point b)
{
    double x = a.x - b.x;
    double y = a.y - b.y;
    return sqrt(x*x+y*y);
}

Point compute_centroid(Point a, Point b)
{
    Point c;
    c.x = (b.x + a.x)/2;
    c.y = (b.y + a.y)/2;
    return c;
}
void draw_2dContour(cv::Mat& image, std::vector<Point2f>& vec_p, cv::Scalar color)
{
  for (size_t i = 0; i < vec_p.size(); i++)
  {
    cv::line(image, vec_p[i], vec_p[ (i+1) % vec_p.size() ], color, 2, CV_AA);
  }
}
void draw_axes(cv::Mat& imageInput, cv::Mat& K, cv::Mat &poseTF, float scale)
{




    cv::Mat R = poseTF(Rect(0,0,3,3));
    cv::Mat t = poseTF(Rect(3,0,1,3));
    cv::Mat Mext = poseTF(Rect(0,0,4,3));

    double Porg_[] = {0, 0, 0};
    cv::Mat Porg = cv::Mat(3,1, CV_64F, Porg_);
    double Px_[] = {scale, 0, 0};
    cv::Mat Px = cv::Mat(3,1, CV_64F, Px_);
    double Py_[] = {0, scale, 0};
    cv::Mat Py = cv::Mat(3,1, CV_64F, Py_);
    double Pz_[] = {0, 0, scale};
    cv::Mat Pz = cv::Mat(3,1, CV_64F, Pz_);
//    double Porg_[] = {0, 0, 0, 1};
//    cv::Mat Porg = cv::Mat(4,1, CV_64F, Porg_);
//    double Px_[] = {1, 0, 0, 1};
//    cv::Mat Px = cv::Mat(4,1, CV_64F, Px_);
//    double Py_[] = {0, 1, 0, 1};
//    cv::Mat Py = cv::Mat(4,1, CV_64F, Py_);
//    double Pz_[] = {0, 0, 1, 1};
//    cv::Mat Pz = cv::Mat(4,1, CV_64F, Pz_);

//    printf("Porg, Px, Py, Pz:\n");
//    printMat(Porg);
//    printMat(Px);
//    printMat(Py);
//    printMat(Pz);

    cv::Mat porg = K*(R*Porg + t);
    cv::Mat px = K*(R*Px + t);
    cv::Mat py = K*(R*Py + t);
    cv::Mat pz = K*(R*Pz + t);

//    cv::Mat porg = K*Mext*Porg;
//    cv::Mat px = K*Mext*Px;
//    cv::Mat py = K*Mext*Py;
//    cv::Mat pz = K*Mext*Pz;


    cv::Point Ptorg(
        (int) (porg.at<double>(0)/porg.at<double>(2)),
        (int) (porg.at<double>(1)/porg.at<double>(2)) );

    cv::Point Pt[3];
    Pt[0] = cv::Point (
        (int) (px.at<double>(0)/px.at<double>(2)),
        (int) (px.at<double>(1)/px.at<double>(2)) );
    Pt[1] = cv::Point (
        (int) (py.at<double>(0)/py.at<double>(2)),
        (int) (py.at<double>(1)/py.at<double>(2)) );
    Pt[2] = cv::Point (
        (int) (pz.at<double>(0)/pz.at<double>(2)),
        (int) (pz.at<double>(1)/pz.at<double>(2)) );

//    for(int i=0;i<3;i++)
//    {
//        double angle;
//        angle = atan2( (double) Ptorg.y - Pt[i].y, (double) Ptorg.x - Pt[i].x );
//        double hypotenuse;
//        hypotenuse = sqrt( square(Ptorg.y - Pt[i].y) + square(Ptorg.x - Pt[i].x) )
//        ;
//        /* Here we lengthen the arrow by a factor of three. */
//         Pt[i].x = (int) (Ptorg.x -  scale*hypotenuse * cos(angle));
//         Pt[i].y = (int) (Ptorg.y -  scale*hypotenuse * sin(angle));
////        std::cout<< Pt[i] << std::endl;
//    }


    line(imageInput, Ptorg, Pt[0], cv::Scalar(0,0,255), 2);
    line(imageInput, Ptorg, Pt[1], cv::Scalar(0,255,0), 2);
    line(imageInput, Ptorg, Pt[2], cv::Scalar(255,0,0), 2);
}
void draw_text(cv::Mat &frame, std::string &text)
{

    Scalar color_state = Scalar(255, 0, 0);
    int fontFace = FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.5;
    int thickness = 2;


    int baseline=0;

    Size textSize = getTextSize(text, fontFace, fontScale, thickness, &baseline);
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
    putText(frame, text, textOrg, fontFace, fontScale,
            color_state, thickness, 8);
}


/** this function was copied from opencv/src/cv/cvundistort.cpp,
    because OpenCV's cvInitUndistortMap silently changed its behavior! **/

// OpenCV copyright information:

/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

void my_cvInitUndistortMap( const CvMat* A, const CvMat* dist_coeffs, CvArr* mapxarr, CvArr* mapyarr )
{
    CV_FUNCNAME( "my_cvInitUndistortMap" );

    __BEGIN__;

    float a[9], k[5]={0,0,0,0,0};
    int coi1 = 0, coi2 = 0;
    CvMat mapxstub, *_mapx = (CvMat*)mapxarr;
    CvMat mapystub, *_mapy = (CvMat*)mapyarr;
    CvMat _a = cvMat( 3, 3, CV_32F, a ), _k;
    int u, v;
    float u0, v0, fx, fy, ifx, ify, x0, y0, k1, k2, k3, p1, p2;
    CvSize size;

    CV_CALL( _mapx = cvGetMat( _mapx, &mapxstub, &coi1 ));
    CV_CALL( _mapy = cvGetMat( _mapy, &mapystub, &coi2 ));

    if( coi1 != 0 || coi2 != 0 )
        CV_ERROR( CV_BadCOI, "The function does not support COI" );

    if( CV_MAT_TYPE(_mapx->type) != CV_32FC1 )
        CV_ERROR( CV_StsUnsupportedFormat, "Both maps must have 32fC1 type" );

    if( !CV_ARE_TYPES_EQ( _mapx, _mapy ))
        CV_ERROR( CV_StsUnmatchedFormats, "" );

    if( !CV_ARE_SIZES_EQ( _mapx, _mapy ))
        CV_ERROR( CV_StsUnmatchedSizes, "" );

    size = cvSize( _mapx->width, _mapy->height ); // m_cvGetMatSize(_mapx);

    if( !CV_IS_MAT(A) || A->rows != 3 || A->cols != 3  ||
        (CV_MAT_TYPE(A->type) != CV_32FC1 && CV_MAT_TYPE(A->type) != CV_64FC1) )
        CV_ERROR( CV_StsBadArg, "Intrinsic matrix must be a valid 3x3 floating-point matrix" );

    if( !CV_IS_MAT(dist_coeffs) || (dist_coeffs->rows != 1 && dist_coeffs->cols != 1) ||
        (dist_coeffs->rows*dist_coeffs->cols*CV_MAT_CN(dist_coeffs->type) != 4 &&
        dist_coeffs->rows*dist_coeffs->cols*CV_MAT_CN(dist_coeffs->type) != 5) ||
        (CV_MAT_DEPTH(dist_coeffs->type) != CV_64F &&
        CV_MAT_DEPTH(dist_coeffs->type) != CV_32F) )
        CV_ERROR( CV_StsBadArg,
            "Distortion coefficients must be 1x4, 4x1, 1x5 or 5x1 floating-point vector" );

    cvConvert( A, &_a );
    _k = cvMat( dist_coeffs->rows, dist_coeffs->cols,
                CV_MAKETYPE(CV_32F, CV_MAT_CN(dist_coeffs->type)), k );
    cvConvert( dist_coeffs, &_k );

    u0 = a[2]; v0 = a[5];
    fx = a[0]; fy = a[4];
    ifx = 1.f/fx; ify = 1.f/fy;
    k1 = k[0]; k2 = k[1]; k3 = k[4];
    p1 = k[2]; p2 = k[3];
    x0 = (size.width-1)*0.5f;
    y0 = (size.height-1)*0.5f;

    for( v = 0; v < size.height; v++ )
    {
        float* mapx = (float*)(_mapx->data.ptr + _mapx->step*v);
        float* mapy = (float*)(_mapy->data.ptr + _mapy->step*v);
        float y = (v - v0)*ify, y2 = y*y;

        for( u = 0; u < size.width; u++ )
        {
            float x = (u - u0)*ifx, x2 = x*x, r2 = x2 + y2, _2xy = 2*x*y;
            double kr = 1 + ((k3*r2 + k2)*r2 + k1)*r2;
            double _x = fx*(x*kr + p1*_2xy + p2*(r2 + 2*x2)) + x0;
            double _y = fy*(y*kr + p1*(r2 + 2*y2) + p2*_2xy) + y0;
            mapx[u] = (float)_x;
            mapy[u] = (float)_y;
        }
    }

    __END__;
}
void undistortFrame(Mat& frame, Mat& matIntrin, Mat& matDist)
{

//    IplImage* mx = cvCreateImage( Size(frame.cols, frame.rows), IPL_DEPTH_32F, 1 );
//    IplImage* my = cvCreateImage( Size(frame.cols, frame.rows), IPL_DEPTH_32F, 1 );

    Mat mx(frame.rows, frame.cols, CV_32FC1);
    Mat my(frame.rows, frame.cols, CV_32FC1);


    CvMat cvmatMx = mx;
    CvMat cvmatMy = my;

    CvMat cvmatIntrin = matIntrin;
    CvMat cvmatDist = matDist;

    my_cvInitUndistortMap( &cvmatIntrin, &cvmatDist, &cvmatMx, &cvmatMy );

    CvMat cvmatFrame = frame;
    cvRemap( &cvmatFrame, &cvmatFrame, &cvmatMx, &cvmatMy); // undistort image

}




}
