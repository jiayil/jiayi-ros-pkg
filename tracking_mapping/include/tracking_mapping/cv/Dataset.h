/* File:    Dataset.h
   Author:  Jiayi Liu
   Date:    May 10, 2013
   Description:


  */
#ifndef DATASET_H
#define DATASET_H
#include <opencv2/opencv.hpp>


class Dataset
{
public:
    Dataset();

    std::vector<cv::Mat> vec_H;
    size_t index;
};

#endif // DATASET_H
