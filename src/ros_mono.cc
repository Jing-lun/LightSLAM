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
#include "Extract.h"
#include "PoseEstimation.h"
#include "RunSLAM.h"
using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(Light_SLAM::RunSLAM* pTest):mpTest(pTest){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    Light_SLAM::RunSLAM* mpTest;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "LightSLAM");
    ros::start();

    string SetiingsFile = "/home/jinglun/viorb_config/config/pointgray_mono.yaml"; 

    Light_SLAM::PoseEstimation2Dto2D PE(SetiingsFile);
    Light_SLAM::ExtractFeature* EF = new Light_SLAM::ExtractFeature;
    Light_SLAM::RunSLAM Test(EF, PE);

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
    // mpSLAM->VisualOdometry(cv_ptr->image,cv_ptr->header.stamp.toSec());
    mpTest->VO(cv_ptr->image);
    // mpTest->ShowFeature(cv_ptr->image);
}


