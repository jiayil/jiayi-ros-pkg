/* This class is a 6-DOF pose.
Bill Hoff   June 2012
*/

#ifndef POSE_H		// header guards 
#define POSE_H

#include <opencv2/opencv.hpp>

class Pose	{

public:
	Pose(cv::Mat& R, cv::Mat& t);
	~Pose();

	// Variables
	cv::Mat	R;	//	3x3 rotation matrix
	cv::Mat	t;	//  3x1 translation vector 

	// Methods
	void getXYZangles(double& ax, double& ay, double& az);
	void getTranslation(double& tx, double& ty, double& tz);
private:

};

#endif

