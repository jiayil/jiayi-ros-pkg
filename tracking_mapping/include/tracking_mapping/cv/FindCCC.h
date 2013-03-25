/* This class handles the task of finding a 5-point CCC target.
Bill Hoff   June 2012
*/

#ifndef FINDCCC_H		// header guards 
#define FINDCCC_H

#include <opencv2/opencv.hpp>

class FindCCC	{

public:
	FindCCC();
	~FindCCC();

	bool process(cv::Mat& imageInputGray, cv::Vec2d targets[]);

private:

};

#endif

