/* This class finds the pose of a CCC target.
The target has the form:
UL UM UR
LL    LR

Bill Hoff   June 2012
*/

#ifndef POSECCC_H		// header guards 
#define POSECCC_H

#include <opencv2/opencv.hpp>

#include "tracking_mapping/cv/Pose.h"

class PoseCCC	{

public:
	PoseCCC();
	~PoseCCC();

	// Points in model coordinates.
	cv::Mat	P1, P2, P3, P4;		// correspond to UL, UR, LL, LR
	cv::Mat	Pcenter;			// the center (or origin)

	Pose getPose(cv::Mat& K, cv::Mat& Kinv, cv::Vec2d targets[]);
private:

};

#endif
