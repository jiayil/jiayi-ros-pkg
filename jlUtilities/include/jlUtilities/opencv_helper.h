#ifndef OPENCV_HELPER_H
#define OPENCV_HELPER_H
#include <opencv2/opencv.hpp>
using namespace cv;
namespace jlUtilities
{
bool sort_feature_response(const KeyPoint &first, const KeyPoint &second);
}
#endif // OPENCV_HELPER_H
