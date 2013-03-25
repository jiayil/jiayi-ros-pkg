/* This class finds a 5-target pattern in an image, consisting of 5 contrasting 
concentric circle targets (CCCs).  The targets are arranged in this order:

UL  UM   UR
LL       LR

Bill Hoff
June 2012
*/

#include <cstdio>
#include <complex>
#include <cmath>
#include "tracking_mapping/cv/FindCCC.h"	// all includes, function prototypes, etc

using namespace std;
using namespace cv;
// Thresholds for finding CCC targets
#define DPIXEL	3.0				// max distance between centroids
#define OUTERMINAREA	75		// min area of outer blob
#define OUTERMAXAREA	5000	// max area of outer blob
#define INNERMINAREA	30		// min area of inner blob
#define INNERMAXAREA 	1000	// max area of inner blob

#define SEETHRESH	0	// Set to 1 to see output thresholded image

// These allow adjustment of threshold (global variables in this file)
int slider_max = 100;
int slider_pos = 10;
double thresh = 1;

// Callback for trackbar (a private function in this file)
void on_trackbar( int, void* )	
{
	thresh = (double) slider_pos;
}

FindCCC::FindCCC()
{
#if SEETHRESH
	// Create Window to see thresholded image
	cv::namedWindow("Threshold Output", CV_WINDOW_AUTOSIZE);

	// Create Trackbar
	cv::createTrackbar("threshold", "Threshold Output", &slider_pos, slider_max, on_trackbar );
	cv::setTrackbarPos("threshold", "Threshold Output", slider_pos);

#endif
}

FindCCC::~FindCCC()
{
}

bool FindCCC::process(cv::Mat& imageInputGray, cv::Vec2d targets[])
{

	// Do adaptive threshold ... this compares each pixel to a local 
	// mean of the neighborhood
	cv::Mat imageThresh;
    cv::adaptiveThreshold(imageInputGray,
		imageThresh,				// output thresholded image
		255,	
		cv::ADAPTIVE_THRESH_MEAN_C,	// local neighborhood
		cv::THRESH_BINARY_INV,		// threshold_type 
		31,							// blockSize
		thresh);					// threshold value

	// Apply morphological operations to clean up
	cv::Mat structuringElmt(3,3,CV_8U,cv::Scalar(1));
	cv::Mat imageOpen;
    cv::morphologyEx(imageThresh, imageOpen, cv::MORPH_OPEN, structuringElmt);

#if SEETHRESH
	imshow("Threshold Output", imageOpen);
#endif



	// Find connected components
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(
		imageOpen,				// input image (is destroyed)
		contours,				// output vector of contours
		hierarchy,				// hierarchical representation
		CV_RETR_CCOMP,			// retrieve all contours 
		CV_CHAIN_APPROX_NONE);	// all pixels of each contours

	// Analyze components and find CCCs
	cv::Vec2d ccc[5];		// CCC targets, unordered
	int nCcc = 0;
	for(int i1 = 0; i1<(int)contours.size(); i1++)	{
		double a1 = contourArea(contours[i1]);
		if (!(a1 > OUTERMINAREA && a1 < OUTERMAXAREA))	continue;

		int i2 = hierarchy[i1][2];
		if (i2 < 0)	continue;			// See if it has a child inside

		double a2 = contourArea(contours[i2]);
		if (!(a2 > INNERMINAREA && a2 < INNERMAXAREA))	continue;

		// Compute centroids of inner and outer regions
		cv::Moments mu1 = cv::moments(contours[i1]);
		cv::Point2d x1(mu1.m10/mu1.m00, mu1.m01/mu1.m00);
		cv::Moments mu2 = cv::moments(contours[i2]);
		cv::Point2d x2(mu2.m10/mu2.m00, mu2.m01/mu2.m00);

		// Check if centroids coincide
		if (norm(x1-x2) > DPIXEL)	continue;

		// This must be a valid target
		if (nCcc < 5)	ccc[nCcc] = x1;
		nCcc++;
	}
	if (nCcc != 5)
		return false;


	// Find the 3 CCCs that are in a line
	int i1,i2,i3;
	double dMin = 1e9;

	for (int i=0; i<5; i++)	{
		for (int j=i+1; j<5; j++)	{
			// Get the mid point between i,j
			cv::Vec2d midPt = (ccc[i]+ccc[j])/2;

			// Find the CCC that is closest to this midpoint
			for (int k=0; k<5; k++)	{
				if (k==i || k==j)	continue;
				double d = norm(ccc[k]-midPt);
				if (d < dMin)	{
					dMin = d;
					i1 = i;
					i2 = k;
					i3 = j;
				}
			}
		}
	}
	if (dMin > 15.0)
		return false;


	// We have found 3 colinear targets:  i1 -- i2 -- i3, now
	// get the indices of the other two, i4, i5
	int i4, i5;
	for (i4=0; i4<5; i4++)
		if (i4!=i1 && i4!=i2 && i4!=i3)	
			break;
	for (i5=0; i5<5; i5++)
		if (i5!=i1 && i5!=i2 && i5!=i3 && i5!=i4)
			break;

	// We may have to switch i4 and i5.  Look at where i4 is with respect to i1,i2,i3.
	// Signed area is the determinant of the 2x2 matrix [ p4-p1, p3-p1 ]
	cv::Vec2d p41 = ccc[i4] - ccc[i1];
	cv::Vec2d p31 = ccc[i3] - ccc[i1];
	double m[2][2] = { {p41[0], p31[0]}, {p41[1], p31[1]} };
	double det = m[0][0]*m[1][1] - m[0][1]*m[1][0];

	// Put the 3 colinear targets in the order UL, UM, UR
	if (det < 0)	{
		targets[0] = ccc[i1];	// UL
		targets[1] = ccc[i2];	// UM
		targets[2] = ccc[i3];	// UR
	} else	{
		targets[0] = ccc[i3];	// UL
		targets[1] = ccc[i2];	// UM
		targets[2] = ccc[i1];	// UR
	}

	// LL is the closer point to UL
	if ( norm(ccc[i4]-targets[0]) < norm(ccc[i5]-targets[0]) )	{
		targets[3] = ccc[i4];	// LL
		targets[4] = ccc[i5];	// LR
	} else	{
		targets[3] = ccc[i5];	// LL
		targets[4] = ccc[i4];	// LR
	}

	return true;
}
