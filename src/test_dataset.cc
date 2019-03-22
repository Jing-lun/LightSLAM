#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iomanip>

#include "VO.h"

int main()
{
    const char *position_file_name = "position.txt";
    const char *frame_viewer_name = "Road facing camera";
    const char *trajectory_viewer_name = "Trajectory";

    const char *image_list_folder = "/home/runisys/Desktop/data/slamDataset/sequences/00/image_0/";
    const char *ground_truth_path = "/home/runisys/Desktop/data/KITTI/odometry/dataset/poses/00.txt";

    char text[200];

    // 针孔相机对象
    int frame_width = 1241;
    int frame_height = 376;
    double fx = 718.8560;
    double fy = 718.8560;
    double cx = 607.1928;
    double cy = 185.2157;

    PinholeCamera camera(frame_width, frame_height, fx, fy, cx, cy);

    // VO对象
    VisualOdometry vo(&camera);
    if(!vo.SetGroundTruth(ground_truth_path))
    {
        std::cout <<"Unable open ground truth file!" <<std::endl;
        return 1;
    }

    // 设置输出坐标文件
    std::fstream out(position_file_name, std::ios::out);

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

    double x = 0, y = 0, z = 0;
    int image_id = 0;

    while (true)
    {
        std::stringstream ss;
        ss << image_list_folder << std::setw(6) << std::setfill('0') << image_id << ".png";

        cv::Mat img = cv::imread(ss.str().c_str(), 0);
        if(img.empty())
            break;

        if(!vo.AddImage(img))
        {
            std::cout << "Vo failed!" << std::endl;
            return 2;
        }
        cv::Mat cur_t = vo.getCurrentT();
        if (cur_t.rows != 0)
        {
            x = cur_t.at<double>(0);
            y = cur_t.at<double>(1);
            z = cur_t.at<double>(2);
        }
        out << x << " " << y << " " << z << std::endl;

        int draw_x = int(x) + 300;
        int draw_y = int(z) + 100;
        cv::circle(traj, cv::Point(draw_x, draw_y), 1, CV_RGB(255, 255, 0), 2);

        cv::rectangle(traj, cv::Point(10, 30), cv::Point(580, 60), CV_RGB(0, 0, 0), CV_FILLED);
        sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", x, y, z);
        cv::putText(traj, text, text_org, font_face, font_scale, cv::Scalar::all(255), thickness, 8);

        cv::imshow(frame_viewer_name, img);
        cv::imshow(trajectory_viewer_name, traj);

        cv::waitKey(1);
        image_id++;
    }
    out.close();

    return 0;
}