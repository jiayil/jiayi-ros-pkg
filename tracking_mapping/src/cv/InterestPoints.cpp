/* This class finds interest points in the image.
Bill Hoff
July 2012
*/
#include "tracking_mapping/cv/InterestPoints.h"	// all includes, function prototypes, etc


#define SEECORNERS	1	// Set to 1 to see output corners

// These allow adjustment of threshold (global variables in this file)
static int slider_max = 100;
static int slider_pos = 10;
static double thresh = 10;

// Callback for trackbar (a private function in this file)
void my_trackbar( int, void* )	
{
	thresh = (double) slider_pos;
}

InterestPoints::InterestPoints()
{
#if SEECORNERS
	// Create Window to see thresholded image
	cv::namedWindow("Corners Output", CV_WINDOW_AUTOSIZE);

	// Create Trackbar
	cv::createTrackbar("threshold", "Corners Output", &slider_pos, slider_max, my_trackbar );
	cv::setTrackbarPos("threshold", "Corners Output", slider_pos);
#endif
}

InterestPoints::~InterestPoints()
{
}

void InterestPoints::process(cv::Mat& imageInputGray, std::vector<cv::KeyPoint>& keypoints)
{
	//// Construction of the Good Feature to Track detector
	//int maxCorners = 50;
	//double quality = 0.01;
	//double minDistance = 10.0;
	//int blockSize = 3;
	//cv::GoodFeaturesToTrackDetector myDetector(
	//	maxCorners,		// maximum number of corners to be returned
	//	quality,		// quality level
	//	minDistance,	// minimum allowed distance between points
	//	blockSize);		// Size of neighborhood	

	// Construction of the FastFeatureDetector
	int threshold = (int) thresh;
	bool nonmaxSuppression = true;
	cv::FastFeatureDetector myDetector(
		threshold,			// Thresh diff between intensity of central pixel and pixels on a circle around it
		nonmaxSuppression);	

	//// Construction of the SURF detector
	//int minHessian = 400;
	//int nOctaves = 4;
	//int nOctaveLayers = 2;
	//bool extended = true;
	//bool upright = false;
	//cv::SurfFeatureDetector myDetector(
	//	minHessian,		// Minimum value for Hessian
	//	nOctaves,		// Number of a gaussian pyramid octaves
	//	nOctaveLayers,	// Number of images within each octave of a gaussian pyramid
	//	extended,		// true=compute the extended descriptors (128 elements each)
	//	upright);		// false=compute orientation of each feature


	// point detection using FeatureDetector method
	myDetector.detect(imageInputGray, keypoints);

#if SEECORNERS
	// Draw keypoints
	cv::Mat	imageOut;
	cv::drawKeypoints(imageInputGray,
		keypoints,
		imageOut,
		cv::Scalar(0,0,255));

	/// Show what you got
	imshow("Corners Output", imageOut );
#endif
}
