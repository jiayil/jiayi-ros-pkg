/* This class implements a 6-DOF pose.

Bill Hoff
June 2012
*/
#include "tracking_mapping/cv/Pose.h"	// all includes, function prototypes, etc

Pose::Pose(cv::Mat& R1, cv::Mat& t1)
{
	R1.copyTo(R);
	t1.copyTo(t);
}

Pose::~Pose()
{
}

// Get rotation in terms of XYZ angle (radians)
void Pose::getXYZangles(double& ax, double& ay, double& az)
{
	double r11 = R.at<double>(0,0);
	double r21 = R.at<double>(1,0);
	double r31 = R.at<double>(2,0);
	double r32 = R.at<double>(2,1);
	double r33 = R.at<double>(2,2);

	ay = atan2( -r31, sqrt( r11*r11 + r21*r21 ) );
	double cy = cos(ay);

	if (fabs(cy) > 1e-10)	{          // some tiny number
		az = atan2( r21/cy, r11/cy );
		ax = atan2( r32/cy, r33/cy );
	} else	{
		// We have a degenerate solution
		double r12 = R.at<double>(1,2);
		double r22 = R.at<double>(2,2);
		az = 0;
		ax = atan2(r12, r22);
		if (ay<0)     ax = -ax;
	}
}

// Get translation parameters
void Pose::getTranslation(double& tx, double& ty, double& tz)
{
	tx = t.at<double>(0);
	ty = t.at<double>(1);
	tz = t.at<double>(2);
}

