/* File:    main.cpp
   Author:  Jiayi Liu
   Date:
   Description:
            The main entry of the system

  */

#include <stdlib.h>
#include <iostream>


#include <ros/ros.h>

#include "tracking_mapping/tm/System.h"

using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracking_mapping");
    ROS_INFO("starting with node name %s", ros::this_node::getName().c_str());

    cout << "  Welcome to Tracking and Mapping " << endl;
    cout << "  --------------- " << endl;
    //    cout << "  Parallel tracking and mapping for Small AR workspaces" << endl;
    //    cout << "  Copyright (C) Isis Innovation Limited 2008 " << endl;
    cout << endl;


    System s;
    s.run();


}
