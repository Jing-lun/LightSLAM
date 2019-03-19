#ifndef EXTRACT_H
#define EXTRACT_H

#include "PoseEstimation.h"
#include <vector>
#include <list>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace Light_SLAM
{

class ExtractFeature
{

public:

    enum FrameState
    {
        INIT_FRAME = 0,
        SECOND_FRAME,
        REGULAR_FRAME
    };

    ExtractFeature();
    ~ExtractFeature(){}

    void DetectFeature(cv::Mat img);
    void ShowFeature(cv::Mat img);
    void ExtractFisrt();
    void ExtractSecond();
    void ExtractRest();

public:

    FrameState framestate;
    cv::Ptr<cv::ORB> mpORB;
    cv::Ptr<cv::DescriptorMatcher> mpMatcher;

    std::vector<cv::KeyPoint> mvKeyPoints;
    cv::Mat mDesp;
        
    std::vector<cv::KeyPoint> mvLastKeyPoints;
    cv::Mat mLastDesp;
    cv::Mat mLastImage;
    cv::Mat mCurrentImage;

    std::vector<cv::DMatch> mvMatches;
    std::vector<cv::DMatch> mvGoodMatches;
};

} //namespace Light_SLAM
#endif // EXTRACT_H
