#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include <vector>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

namespace Light_SLAM
{

class VisualOdometry
{

public:

    enum FrameState
    {
        INIT_FRAME,
        SECOND_FRAME,
        LOST_FRAME
    };

    VisualOdometry(const std::string &strSettingPath);
    ~VisualOdometry(){}

    void DetectFeature(cv::Mat img);
    void ShowFeature(cv::Mat img);
    void ExtractFisrt();
    void ExtractSecond();
    void ExtractRest();
    void CalEssential();

public:

    FrameState framestate;
    cv::Ptr<cv::ORB> mpORB;
    // cv::Ptr<cv::DescriptorMatcher> mpMatcher;
    cv::FlannBasedMatcher mMatcher;
    std::vector<cv::KeyPoint> mvKeyPoints;
    cv::Mat mDesp;
        
    std::vector<cv::KeyPoint> mvLastKeyPoints;
    cv::Mat mLastDesp;
    cv::Mat mLastImage;
    cv::Mat mCurrentImage;

    std::vector<cv::DMatch> mvMatches;
    std::vector<cv::DMatch> mvGoodMatches;

    cv::Mat mK;
    cv::Point2d mOpticalCenter;
    int mFocalLength;
    cv::Mat mR;
    cv::Mat mt;
};

} //namespace Light_SLAM
#endif // VISUALODOMETRY_H
