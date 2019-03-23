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
#include "VO.h"

int main()
{
    const char *position_file_name = "position.txt";
    const char *frame_viewer_name = "Road facing camera";
    const char *trajectory_viewer_name = "Trajectory";
    const std::string SetiingsFile = "/home/jinglun/viorb_config/config/pointgray_mono.yaml"; 
    const std::string image_list_folder = "/home/jinglun/Downloads/data_odometry_gray/00/image_0/";
    const std::string ground_truth_path = "/home/jinglun/Documents/dataset/poses/00.txt";

    char text[200];

    // VO对象
    Light_SLAM::VO* EF = new Light_SLAM::VO(SetiingsFile, ground_truth_path);

    // 定义显示轨迹的参数
    int font_face = cv::FONT_HERSHEY_PLAIN;
    double font_scale = 1;
    int thickness = 1;
    cv::Point text_org(10, 50);

    // 定义显示帧窗口和显示轨迹窗口
    cv::namedWindow(frame_viewer_name, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(trajectory_viewer_name, cv::WINDOW_AUTOSIZE);
    // 定义显示轨迹的视图
    cv::Mat traj = cv::Mat::zeros(600, 600, CV_8UC3);
    std::fstream out(position_file_name, std::ios::out);

    double x = 0, y = 0, z = 0; 

    while (true)
    {
        std::stringstream ss;
        ss << image_list_folder << std::setw(6) << std::setfill('0') << frame_id << ".png";

        cv::Mat img = cv::imread(ss.str().c_str(), 0);
        if(img.empty())
            break;
            
        EF->ProcessFrames(img);
        cv::Mat t = EF->getTranslation();
        double x = 0, y = 0, z = 0;   
        if(t.rows != 0)
        {
            x = t.at<double>(0);
            y = t.at<double>(1);
            z = t.at<double>(2);
        }
        out << x << " " << y << " " << z << std::endl;

        int draw_x = static_cast<int>(x) + 300;
        int draw_y = static_cast<int>(z) + 100;
        cv::circle(traj, cv::Point(draw_x, draw_y), 0.6, CV_RGB(255, 0, 220), 2);

        cv::rectangle(traj, cv::Point(10, 30), cv::Point(580, 60), CV_RGB(0, 0, 0), CV_FILLED);
        sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", x, y, z);
        cv::putText(traj, text, text_org, font_face, font_scale, cv::Scalar::all(255), thickness, 8);

        cv::imshow("Camera", img);
        cv::imshow("Trajectory", traj);

        cv::waitKey(1);
        frame_id++;
    }
    out.close();

    return 0;
}