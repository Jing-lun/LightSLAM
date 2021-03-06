#include "RunSLAM.h"
#include <iostream>
using namespace std;
using namespace cv;

namespace Light_SLAM
{
    RunSLAM::RunSLAM(VisualOdometry* pVO):mpTest1(pVO), mFontFace(FONT_HERSHEY_PLAIN), mFontScale(1.0), 
                                        mFontThickness(1), mText(10, 50)
    {
        namedWindow("Camera");
        namedWindow("Trajectory");
        mTrajectory = Mat::zeros(800, 800, CV_8UC3);
        out = fstream("position_orb.txt", ios::out);
    }

    // RunSLAM::RunSLAM(VO* pVO):mpTest1(pVO), mFontFace(FONT_HERSHEY_PLAIN), mFontScale(1.0), 
    //                                     mFontThickness(1), mText(10, 50)
    // {
    //     namedWindow("Camera");
    //     namedWindow("Trajectory");
    //     mTrajectory = Mat::zeros(600, 600, CV_8UC3);
    //     mpTest1->SetGroundTruth();
    // }

    void RunSLAM::vo(const cv::Mat& img)
    {
        mpTest1->ProcessFrames(img);
        cv::Mat t = mpTest1->getTranslation();
        double x = 0, y = 0, z = 0;   
        if(t.rows != 0)
        {
            x = t.at<double>(0);
            y = t.at<double>(1);
            z = t.at<double>(2);
        }
        out << x << " " << y << " " << z << endl;

        int draw_x = static_cast<int>(x) + 400;
        int draw_y = static_cast<int>(z) + 100;
        cv::circle(mTrajectory, cv::Point(draw_x, draw_y), 1, CV_RGB(255, 0, 220), 2);

        cv::rectangle(mTrajectory, cv::Point(10, 30), cv::Point(600, 60), CV_RGB(0, 0, 0), CV_FILLED);
        sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", x, y, z);
        cv::putText(mTrajectory, text, mText, mFontFace, mFontScale, cv::Scalar::all(255), mFontThickness, 8);

        cv::imshow("Camera", img);
        cv::imshow("Trajectory", mTrajectory);

        cv::waitKey(1);       
    }

    RunSLAM::~RunSLAM()
    {
        if (this->out.is_open())
            this->out.close();
        try
        {     
            cv::imwrite("Trajectory.png", mTrajectory);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }
}