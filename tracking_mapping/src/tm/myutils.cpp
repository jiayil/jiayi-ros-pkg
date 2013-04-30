#include <cstdio>
#include "tracking_mapping/tm/myutils.h"
// Scale an image to 0..255 and put it into an image of type CV_8U

namespace myutils
{
void scaleImageTo8U(cv::Mat& imageInput, cv::Mat& imageOutput)
{
	double r1, r2;
	minMaxLoc(imageInput, &r1, &r2);
	//printf("minimum value = %f, maximum value = %f\n", r1, r2);
	imageInput.convertTo(imageOutput, CV_8U,
		255/(r2-r1),
		-(255*r1)/(r2-r1));
}

// Scale an image to 0..255 and put it into an image of type CV_8UC3
void scaleImageTo8UC3(cv::Mat& imageInput, cv::Mat& imageOutput)
{
	double r1, r2;
	cv::Mat imageTemp;

	minMaxLoc(imageInput, &r1, &r2);
	imageInput.convertTo(imageTemp, CV_8U,
		255/(r2-r1),
		-(255*r1)/(r2-r1));
	cvtColor(imageTemp, imageOutput, CV_GRAY2BGR);
}

// Print a matrix
void printMat(cv::Mat& m)
{
	for (int r=0; r<m.rows; r++)	{
		for (int c=0; c<m.cols; c++)
			std::cout << " " << m.at<double>(r,c);
		std::cout << std::endl;
	}
	std::cout << std::endl;
}

// Draw model coordinate axes 
void drawAxes(cv::Mat& imageInput, cv::Mat& K, Pose pose)
{
	double Porg_[] = {0, 0, 0};
	cv::Mat Porg = cv::Mat(3,1, CV_64F, Porg_);
	double Px_[] = {1, 0, 0};
	cv::Mat Px = cv::Mat(3,1, CV_64F, Px_);
	double Py_[] = {0, 1, 0};
	cv::Mat Py = cv::Mat(3,1, CV_64F, Py_);
	double Pz_[] = {0, 0, 1};
	cv::Mat Pz = cv::Mat(3,1, CV_64F, Pz_);

//    printf("Porg, Px, Py, Pz:\n");
//    printMat(Porg);
//    printMat(Px);
//    printMat(Py);
//    printMat(Pz);

	cv::Mat porg = K*(pose.R*Porg + pose.t);
	cv::Mat px = K*(pose.R*Px + pose.t);
	cv::Mat py = K*(pose.R*Py + pose.t);
	cv::Mat pz = K*(pose.R*Pz + pose.t);
	cv::Point Ptorg( 
		(int) (porg.at<double>(0)/porg.at<double>(2)),
		(int) (porg.at<double>(1)/porg.at<double>(2)) );
	cv::Point Ptx( 
		(int) (px.at<double>(0)/px.at<double>(2)),
		(int) (px.at<double>(1)/px.at<double>(2)) );
	cv::Point Pty( 
		(int) (py.at<double>(0)/py.at<double>(2)),
		(int) (py.at<double>(1)/py.at<double>(2)) );
	cv::Point Ptz( 
		(int) (pz.at<double>(0)/pz.at<double>(2)),
		(int) (pz.at<double>(1)/pz.at<double>(2)) );

	line(imageInput, Ptorg, Ptx, cv::Scalar(0,0,255), 2);
	line(imageInput, Ptorg, Pty, cv::Scalar(0,255,0), 2);
	line(imageInput, Ptorg, Ptz, cv::Scalar(255,0,0), 2);
}


void putStatus(cv::Mat &img, std::string & str)
{
    Scalar color_state = Scalar(255, 0, 0);
    int fontFace = FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.5;
    int thickness = 1;
    int baseline=0;
    Size textSize = getTextSize(str, fontFace, fontScale, thickness, &baseline);
    baseline += thickness;
    // center the text
    Point textOrg(0, (img.rows - textSize.height/2));
    // draw the box
    //        rectangle(frame, textOrg + Point(0, baseline),
    //                  textOrg + Point(textSize.width, -textSize.height),
    //                  Scalar(0,0,255));
    // ... and the baseline first
    //        line(frame, textOrg + Point(0, thickness),
    //             textOrg + Point(textSize.width, thickness),
    //             Scalar(0, 0, 255));
    // then put the text itself
    putText(img, str, textOrg, fontFace, fontScale,
            color_state, thickness, 8);
}


}
