#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

int main()
{
    VideoCapture cap(1);
    if(!cap.isOpened())  // check if we succeeded
        return -1;
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 800);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 600);
    VideoWriter writer("VideoTest.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25.0, Size(800, 600));
    Mat frame;
    char key;

    while (1)
    {
        cap >> frame;
        writer << frame;
        imshow("video", frame);

        key = waitKey(30);
        if ( key == 27)
        {
            break;
        }
    }
    std::cout<< "Finish recording." <<std::endl;
    return 0;
}
