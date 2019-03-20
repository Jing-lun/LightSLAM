#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <chrono>
#include <string>

#include <opencv2/core/core.hpp>
#include "VisualOdometry.h"
#include "RunSLAM.h"

int main()
{
    const char *position_file_name = "position.txt";
    const char *frame_viewer_name = "Road facing camera";
    const char *trajectory_viewer_name = "Trajectory";

    const char *image_list_folder = "/home/jinglun/Downloads/data_odometry_gray/00/image_0/";
    const char *ground_truth_path = "/home/jinglun/Documents/dataset/poses/00.txt";
    const char *SetiingsFile = "/home/jinglun/viorb_config/config/pointgray_mono.yaml"; 

    Light_SLAM::VisualOdometry* EF = new Light_SLAM::VisualOdometry(SetiingsFile);
    Light_SLAM::RunSLAM Test(EF);
    double x = 0, y = 0, z = 0;
    int image_id = 0;


    while (true)
    {
        std::stringstream ss;
        ss << image_list_folder << std::setw(6) << std::setfill('0') << image_id << ".png";

        cv::Mat img = cv::imread(ss.str().c_str(), 0);
        if(img.empty())
            break;
        
        Test.VO(img);
        image_id++;        
    }

    return 0;
}