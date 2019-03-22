/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include "VisualOdometry.h"
#include "VO.h"
#include "RunSLAM.h"
using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(Light_SLAM::RunSLAM* pTest):mpTest(pTest){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    Light_SLAM::RunSLAM* mpTest;
};

int main(int argc, char** argv)
{
    if(argc != 4)
    {
        std::cerr << std::endl << "Usage: rosrun light_slam light_slam path_to_settings" << std::endl;    
        ros::shutdown();    
        return 1;
    }
    const char *SetiingsFile = argv[1];
    
    ros::init(argc, argv, "LightSLAM");
    ros::start();

    // Light_SLAM::VisualOdometry* EF = new Light_SLAM::VisualOdometry(SetiingsFile);
    Light_SLAM::VO* EF = new Light_SLAM::VO(SetiingsFile);
    Light_SLAM::RunSLAM Test(EF);

    ImageGrabber igb(&Test);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

    ros::spin();

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    mpTest->vo(cv_ptr->image);
}


