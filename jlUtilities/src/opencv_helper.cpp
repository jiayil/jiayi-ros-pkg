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
}
