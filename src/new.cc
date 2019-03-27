#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iomanip>
#include <thread>
#include <algorithm>
#include <chrono>

#include <opencv2/core/core.hpp>
#include "VO.h"

// #define RED cv::Scalar(0, 0, 255)
// #define BLUE cv::Scalar(255, 0, 0)

void load_pose(const std::string& path, std::vector<std::vector<double>>& poses);

int main()
{
    const std::string testing_path = "/home/jinglun/slam_ws/src/light_slam/position_orb.txt";
    // const std::string testing_path = "/home/jinglun/slam_ws/devel/lib/light_slam/position.txt";
    const std::string ground_truth_path = "/home/jinglun/Documents/dataset/poses/00.txt";
    const char *frame_viewer_name = "Road facing camera";
    const char *trajectory_viewer_name = "Trajectory";

    char test_text[200];
    char truth_text[200];
    // Define trajectory params
    int font_face = cv::FONT_HERSHEY_PLAIN;
    double font_scale = 1;
    int thickness = 1;
    cv::Point test_org(10, 50);
    cv::Point truth_org(10, 70);

    std::vector<std::vector<double>> positions;
    load_pose(testing_path, positions);

    std::vector<std::vector<double>> poses;
    load_pose(ground_truth_path, poses);

    // Define show windows
    cv::namedWindow(frame_viewer_name, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(trajectory_viewer_name, cv::WINDOW_AUTOSIZE);
    // Define traj map
    cv::Mat traj = cv::Mat::zeros(800, 800, CV_8UC3);

    double x = 0, y = 0, z = 0; 
    double xt = 0, yt = 0, zt = 0;
    int size = positions.size();

    for(int num=0; num<size; num++)
    {
        std::vector<double> position;
        position = positions[num];
        x = position[0];
        y = position[1];
        z = position[2];        

        int draw_x = static_cast<int>(x) + 300;
        int draw_y = static_cast<int>(z) + 200;

        std::vector<double> pose;
        pose = poses[num];
        xt = pose[3];
        yt = pose[7];
        zt = pose[11];        

        int draw_xt = static_cast<int>(xt) + 300;
        int draw_yt = static_cast<int>(zt) + 200;

        cv::rectangle(traj, cv::Point(10, 30), cv::Point(600, 100), CV_RGB(0, 0, 0), CV_FILLED);

        cv::circle(traj, cv::Point(draw_x, draw_y), 0.6, cv::Scalar(255, 0, 255), 2);
        sprintf(test_text, "Test Position: x = %02fm y = %02fm z = %02fm", x, y, z);
        cv::putText(traj, test_text, test_org, font_face, font_scale, cv::Scalar(255, 0, 255), thickness, 8);

        cv::circle(traj, cv::Point(draw_xt, draw_yt), 0.6, cv::Scalar(255, 255, 0), 2);
        sprintf(truth_text, "Truth Position: x = %02fm y = %02fm z = %02fm", xt, yt, zt);
        cv::putText(traj, truth_text, truth_org, font_face, font_scale, cv::Scalar(255, 255, 0), thickness, 8);

        // cv::imshow("Camera", img);
        cv::imshow("Trajectory", traj);
        cv::waitKey(1);
    }

    // for(int num=0; num<size2; num++)
    // {
    //     std::vector<double> pose;
    //     pose = poses[num];
    //     xt = pose[3];
    //     yt = pose[7];
    //     zt = pose[11];        

    //     int draw_xt = static_cast<int>(xt) + 300;
    //     int draw_yt = static_cast<int>(zt) + 100;
    //     cv::circle(traj, cv::Point(draw_xt, draw_yt), 0.6, CV_RGB(0, 0, 255), 2);
    //     sprintf(truth_text, "Coordinates: x = %02fm y = %02fm z = %02fm", xt, yt, zt);
    //     cv::putText(traj, truth_text, truth_org, font_face, font_scale, cv::Scalar(0, 0, 255), thickness, 8);

    //     // cv::imshow("Camera", img);
    //     cv::imshow("Trajectory", traj);
    //     cv::waitKey(1);
    // }

    try
    {     
        cv::imwrite("Trajectory.png", traj);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    return 0;
}

void load_pose(const std::string& path, std::vector<std::vector<double>>& poses)
{
    std::cout << "Loading path file:" << path << std::endl;
    std::ifstream myfile(path);
    std::string line;
    if(myfile.is_open())
    {
        while(getline(myfile, line))
        {
            std::stringstream ss;
            ss << line;
            std::vector<double> v;
            v.resize(12);
            for(int i=0; i<12; i++)
            {
                double value;
                ss >> value;
                v[i] = value;
            }
            poses.push_back(v);
        }    
        myfile.close();    
    }
    else
    {
        std::cout << "Unable to open file" << std::endl;
    }
    
}