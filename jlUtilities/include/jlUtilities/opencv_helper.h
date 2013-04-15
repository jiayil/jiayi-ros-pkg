#ifndef OPENCV_HELPER_H
#define OPENCV_HELPER_H
#include <opencv2/opencv.hpp>
using namespace cv;
namespace jlUtilities
{
bool sort_feature_response(const KeyPoint &first, const KeyPoint &second);
double compute_magnitude(Point a, Point b);
Point compute_centroid(Point a, Point b);
void draw_2dContour(cv::Mat& image, std::vector<Point2f>& vec_p, cv::Scalar color);
}
#endif // OPENCV_HELPER_H
