#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <boost/lexical_cast.hpp>

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
    bool flag_record = false;

    std::string imgName_prefix = "data/image_0_0_";
    std::string imgName_suffix = "_0_0_0.jpg";
    std::string imgName;
    size_t imgName_cnt = 80;

    while (1)
    {
        cap >> frame;

        imshow("video", frame);

        key = waitKey(30);
        if ( key == 27)
        {
            break;
        }
        else if(key == 'r')
        {
            flag_record = true;
            printf("Start recording...\n");
        }
        else if(key == 'e')
        {
            flag_record = false;
            printf("End recording...\n");
        }
        else if(key == 's')
        {
            imgName = imgName_prefix +
                    boost::lexical_cast<std::string>(imgName_cnt) +
                    imgName_suffix;
            imgName_cnt += 5;
            imwrite( imgName, frame );
            printf("Take snapshot %s ... Done\n", imgName.c_str());
        }
        if(flag_record == true)
            writer << frame;
    }
    std::cout<< "Finish recording." <<std::endl;
    return 0;
}
