/* This class handles the task of finding interest points.
Bill Hoff   July 2012
*/

#ifndef INTERESTPOINTS_H		// header guards 
#define INTERESTPOINTS_H

#include <opencv2/opencv.hpp>

class InterestPoints	{

public:
	InterestPoints();
	~InterestPoints();

	// Methods
	void process(cv::Mat& imageInputGray, 
		std::vector<cv::KeyPoint>& keypoints);

private:

};

#endif
