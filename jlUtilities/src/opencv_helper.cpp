/* File:    opencv_helper.cpp
   Author:  Jiayi Liu
   Date:    Mar 17, 2013
   Description:


  */

#include "jlUtilities/opencv_helper.h"

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
    c.x = (b.x - a.x)/2;
    c.y = (b.y - b.x)/2;
    return c;
}
void draw_2dContour(cv::Mat& image, std::vector<Point2f>& vec_p, cv::Scalar color)
{
  for (size_t i = 0; i < vec_p.size(); i++)
  {
    cv::line(image, vec_p[i], vec_p[ (i+1) % vec_p.size() ], color, 2, CV_AA);
  }
}
}
