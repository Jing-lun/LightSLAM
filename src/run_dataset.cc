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
// #include "VO.h"
#include "RunSLAM.h"

int main(int argc, char* argv[])
{
    if(argc != 4)
    {
        std::cout << argc;
        std::cerr << std::endl << "Usage: please add path_to_dataset_image path_to_dataset_groundtruth path_to_settings" << std::endl;        
        return 1;
    }
    const std::string image_list_folder = argv[1];
    const std::string ground_truth_path = argv[2];
    const std::string SetiingsFile = argv[3]; 

    Light_SLAM::VisualOdometry* EF = new Light_SLAM::VisualOdometry(SetiingsFile, ground_truth_path);
    // Light_SLAM::VO* EF = new Light_SLAM::VO(SetiingsFile, ground_truth_path);
    Light_SLAM::RunSLAM Test(EF);

    while (true)
    {
        std::stringstream ss;
        ss << image_list_folder << std::setw(6) << std::setfill('0') << frame_id << ".png";

        cv::Mat img = cv::imread(ss.str().c_str(), 0);
        if(img.empty())
            break;
        
        // EF->ProcessFrames(img);
        Test.vo(img);
        frame_id++;        
    }

    return 0;
}