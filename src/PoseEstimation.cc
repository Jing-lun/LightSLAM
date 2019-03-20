#include <vector>
#include <iostream>
#include "PoseEstimation.h"
#include <cstdlib>

using namespace cv;
using namespace std;

namespace Light_SLAM
{
    PoseEstimation2Dto2D::PoseEstimation2Dto2D(const string &strSettingPath)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ); //Read Setting.yaml
        //     |fx  0   cx|
	    // K = |0   fy  cy|
	    //     |0   0   1 |
	    float fx = fSettings["Camera.fx"];
	    float fy = fSettings["Camera.fy"];
	    float cx = fSettings["Camera.cx"];
	    float cy = fSettings["Camera.cy"];
	    cv::Mat K = cv::Mat::eye(3,3,CV_32F);// initialized as diagnal matrix
	    K.at<float>(0,0) = fx;
	    K.at<float>(1,1) = fy;
	    K.at<float>(0,2) = cx;
	    K.at<float>(1,2) = cy;
	    K.copyTo(mK); 
        // mOpticalCenter.x = cx;
        // mOpticalCenter.y = cy;
        mOpticalCenter = cv::Point2d(cx, cy);
        mFocalLength = static_cast<double>(fx);
    }

    void PoseEstimation2Dto2D::CalculateEssential(std::vector<cv::KeyPoint> &keypoints1, std::vector<cv::KeyPoint> &keypoints2,
                            std::vector<cv::DMatch> &matches, const cv::Mat &R, const cv::Mat &t)
    {
        vector<Point2f> points1;
        vector<Point2f> points2;
        Mat mask, E;

        for ( int i = 0; i < ( int ) matches.size(); i++ )
        {
            points1.push_back ( keypoints1[matches[i].queryIdx].pt );
            points2.push_back ( keypoints2[matches[i].trainIdx].pt );
        }

        // cv::Mat F = cv::findFundamentalMat(points1, points2, CV_FM_8POINT);
        // std::cout << "Fundamental matrix is " << std::endl << F << std::endl;

        E = findEssentialMat(points1, points2, mFocalLength, mOpticalCenter, RANSAC, 0.999, 1.0, mask);
        cout << "Essential matrix is " << endl << E << endl;
        // try
        // {
            recoverPose(E, points1, points2, R, t, mFocalLength, mOpticalCenter, mask);        
            R.copyTo(mR);
            t.copyTo(mt);
            cout << "Rotation matrix is " << endl << R << endl;
            cout << "transform matrix is " << endl << t << endl;
        // }
        // catch(const std::exception& e)
        // {
        //     std::cerr << e.what() << '\n';
        // }
        
        
    }
}