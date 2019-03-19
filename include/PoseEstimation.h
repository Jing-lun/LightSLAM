#ifndef POSEESTIMATION_H
#define POSEESTIMATION_H

#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include "Extract.h"

namespace Light_SLAM
{

class ExtractFeature;

class PoseEstimation2Dto2D
{

public:

    PoseEstimation2Dto2D(const std::string &strSettingPath);
    ~PoseEstimation2Dto2D(){}

    void CalculateEssential(std::vector<cv::KeyPoint> &keypoints1, std::vector<cv::KeyPoint> &keypoints2,
                            std::vector<cv::DMatch> &matches, const cv::Mat &R, const cv::Mat &t);
    
    cv::Mat mK;
    cv::Point2d mOpticalCenter;
    int mFocalLength;
    cv::Mat mR;
    cv::Mat mt;
};

} // namespace Light_SLAM
#endif // POSEESTIMATION_H