#ifndef OPENCV_HELPER_H
#define OPENCV_HELPER_H
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <sstream>
using namespace cv;
namespace jlUtilities
{
bool sort_feature_response(const KeyPoint &first, const KeyPoint &second);
double compute_magnitude(Point a, Point b);
Point compute_centroid(Point a, Point b);
void draw_2dContour(cv::Mat& image, std::vector<Point2f>& vec_p, cv::Scalar color);
void draw_axes(cv::Mat& imageInput, cv::Mat& K, cv::Mat &poseTF, float scale=1);
void draw_text(Mat &frame, std::string &text);
template<typename T>
void draw_pointCoord(Mat &frame, T &pIn)
{
    Point p = Point((int)pIn.x, (int)pIn.y);
    cv::circle(frame,
               p,	// center
               3,							// radius
        cv::Scalar(0,0,255),		// color
        -1);
    std::ostringstream out;
    out << "(" << p.x << ", " << p.y << ")";

    putText (frame, out.str(), p,
        cv::FONT_HERSHEY_PLAIN, // font face
        1.0,					// font scale
        cv::Scalar(255,0,0),	// font color
        1);						// thickness
}
template<typename T>
void draw_pointCoord(Mat &frame, std::vector<T> &pts)
{
    for(int i=0;i<pts.size();i++)
    {
        draw_pointCoord(frame, pts[i]);
    }
}
void my_cvInitUndistortMap( const CvMat* A, const CvMat* dist_coeffs, CvArr* mapxarr, CvArr* mapyarr );
void undistortFrame(Mat& frame, Mat& matIntrin, Mat& matDist);
}
#endif // OPENCV_HELPER_H
