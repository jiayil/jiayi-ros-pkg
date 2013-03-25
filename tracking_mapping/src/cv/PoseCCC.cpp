/* This class finds the pose of a CCC target.

Bill Hoff
June 2012
*/
#include "tracking_mapping/cv/PoseCCC.h"	// all includes, function prototypes, etc

PoseCCC::PoseCCC()
{
	// Points in model coordinates
	double UL_[3] = {0, 0, 0};
	double UM_[3] = {3.7, 0, 0};
	double UR_[3] = {7.4, 0, 0};
	double LL_[3] = {0, 4.55, 0};
	double LR_[3] = {7.4, 4.55, 0};
	P1 = cv::Mat(3,1, CV_64F, UL_).clone();
	P2 = cv::Mat(3,1, CV_64F, UR_).clone();
	P3 = cv::Mat(3,1, CV_64F, LL_).clone();
	P4 = cv::Mat(3,1, CV_64F, LR_).clone();

	// This point is considered to be the center (or origin)
	double Pcenter_[3] = {3.7, 4.55/2, 0};
	Pcenter = cv::Mat(3,1, CV_64F, Pcenter_).clone();
}

PoseCCC::~PoseCCC()
{
}


// Compute the pose of the targets, given the image positions.
// The input is the array of CCC target locations.
//   target[0] = UL, target[1] = UM, target[2] = UR, target[3] = LL, target[4] = LR
// We will only use UL, UR, LL, LR to compute the pose.
Pose PoseCCC::getPose(cv::Mat& K, cv::Mat& Kinv, cv::Vec2d targets[])
{
	// Input points
	double cTarget_[3][4] = 
	{ {targets[0][0],	targets[2][0],	targets[3][0],	targets[4][0]},
	{targets[0][1],	targets[2][1],	targets[3][1],	targets[4][1]},
	{1, 1, 1, 1} };
	cv::Mat cTarget = cv::Mat(3,4, CV_64F, cTarget_).clone();

	//printf("Image coordinates of the four targets:\n");
	//printMat(cTarget);

	// Normalized image points
	cv::Mat p1 = Kinv*cTarget.col(0);
	cv::Mat p2 = Kinv*cTarget.col(1);
	cv::Mat p3 = Kinv*cTarget.col(2);
	cv::Mat p4 = Kinv*cTarget.col(3);

	//printf("Normalized image coordinates of the four targets:\n");
	//printMat(p1);
	//printMat(p2);
	//printMat(p3);
	//printMat(p4);

	//printf("Model coordinates of the four targets:\n");
	//printMat(P1);
	//printMat(P2);
	//printMat(P3);
	//printMat(P4);

	double d12 = norm(P2-P1);  // distance from 1 to 2
	cv::Mat u12 = (P2-P1)/d12;  // unit vector from 1 to 2
	double d13 = norm(P3-P1);  // distance from 1 to 3
	cv::Mat u13 = (P3-P1)/d13;  // unit vector from 1 to 3

	//printf("d12=%f, d13=%f\n", d12, d13);
	//printf("u12, u13:\n");
	//printMat(u12);
	//printMat(u13);

	// Point 4 can be expressed in terms of the other 3 points;
	// ie (P4-P1) = a*u12 + b*u13
	double a = u12.dot(P4-P1);
	double b = u13.dot(P4-P1);
	//printf("a=%f, b=%f\n", a, b);

	// Let P1=k1*p1, P2=k2*p2, P3=k3*p3, P4=k4*p4.
	// Then  P4-P1 = a*u12 + b*u13 = a*(P2-P1)/d12 + b*(P3-P1)/d13
	// or    k4*p4-k1*p1 = (a/d12)*(k2*p2-k1*p1) + (b/d13)*(k3*p3-k1*p1)
	//       m4*p4-p1 = (a/d12)*(m2*p2-p1) + (b/d13)*(m3*p3-p1)
	//  where m4=k4/k1, m3=k3/k1, m2=k2/k1
	// Collecting terms:
	// (a/d12)*m2*p2 + (b/d13)*m3*p3 - m4*p4 = (a/d12)*p1 + (b/d13)*p1 - p1
	cv::Mat A = cv::Mat(3,3, CV_64F);
	A.col(0) = (a/d12)*p2;
	A.col(1) = (b/d13)*p3;
	A.col(2) = -p4;
	cv::Mat c = (a/d12)*p1 + (b/d13)*p1 - p1;
	cv::Mat m = A.inv(cv::DECOMP_SVD)*c;	// m = [m2;m3;m4]

     // Assume P1,P2 define X axis
	cv::Mat ux = (m.at<double>(0)*p2 - p1)/norm(m.at<double>(0)*p2 - p1);
    // Assume P1,P3 define Y axis
	cv::Mat uy = (m.at<double>(1)*p3 - p1)/norm(m.at<double>(1)*p3 - p1); 

    cv::Mat uz = ux.cross(uy);

	// R = [ux uy uz];
	cv::Mat R = cv::Mat(3,3, CV_64F);
	ux.copyTo(R.col(0));
	uy.copyTo(R.col(1));
	uz.copyTo(R.col(2));

	// Force R to be a valid rotation matrix
	cv::Mat S, U, Vt;
    cv::SVD::compute(R, S, U, Vt);						// [U,S,V] = svd(R);
	cv::Mat R_m_c = U*(cv::Mat::eye(3,3,CV_64F))*Vt;	// R_m_c = U*eye(3)*V';

	double k1 = d12/norm(m.at<double>(0)*p2 - p1);	// Distance to point P1
    cv::Mat P1_c = k1*p1;       // Point 1 in camera coordinates

	//printf("Distance to point P1 = %f\n", k1);
	//printf("Point 1 in camera coordinates:\n");
	//printMat(P1_c);

	// We want the pose with respect to the point Pcenter, not P1.
	// Subtract Pcenter from P1.
    cv::Mat tmorg_c = P1_c - R_m_c * (P1-Pcenter);

	//printf("Final pose (R,t):\n");
	//printMat(R_m_c);
	//printMat(tmorg_c);

	Pose pose(R_m_c, tmorg_c);
	return pose;
}
