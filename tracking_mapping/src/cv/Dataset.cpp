/* File:    Dataset.cpp
   Author:  Jiayi Liu
   Date:    Maay 10, 2013
   Description:


  */

#include <fstream>
#include "tracking_mapping/cv/Dataset.h"

using namespace std;
Dataset::Dataset()
{

    std::string fibuzm =
            "/home/jiayi/Research/Dataset/UCSB/groundtruth_warps/fi-bu-zm.avi.warps";
    std::string fiburt =
            "/home/jiayi/Research/Dataset/UCSB/groundtruth_warps/fi-bu-rt.avi.warps";
    std::string fibupd =
            "/home/jiayi/Research/Dataset/UCSB/groundtruth_warps/fi-bu-pd.avi.warps";
    std::string fibuuc =
            "/home/jiayi/Research/Dataset/UCSB/groundtruth_warps/fi-bu-uc.avi.warps";

    std::string fileName = fibuuc;
    ifstream myfile;
    myfile.open(fileName.c_str());
    if (!myfile.is_open())
    {
        printf("Dataset: Error! Failed to open file: %s\n", fileName.c_str());
    }
    else
    {
        std::string   line;
        while ( std::getline(myfile, line) )
        {
            double M_[3][3];
            std::stringstream  lineStream(line);

            for(int i=0;i<3;i++)
                for(int j=0;j<3;j++)
                {
                    lineStream >> M_[i][j];
                }
            vec_H.push_back(cv::Mat(3, 3, CV_64F, M_).clone());
//            printf("Elem: %f\n.", M_[0][0]);
        }
        myfile.close();
    }
    index = 0;
    printf("Dataset is constructed. Size: %d\n", vec_H.size());

}
