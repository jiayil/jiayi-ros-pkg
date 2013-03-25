/* File:    Initializer.h
   Author:  Jiayi Liu
   Date:    Mar 25, 2013
   Description:


  */
#ifndef INITIALIZER_H
#define INITIALIZER_H

#include "tracking_mapping/cv/FindCCC.h"
#include "tracking_mapping/cv/InterestPoints.h"
#include "tracking_mapping/cv/Pose.h"
#include "tracking_mapping/cv/PoseCCC.h"


class Initializer
{
public:
    Initializer();

    FindCCC findCCC;
    PoseCCC poseCCC;
    InterestPoints interestPoints;


};

#endif // INITIALIZER_H
