#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include <vector>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

extern int frame_id;
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
    VisualOdometry(const std::string &strSettingPath, const std::string &strGroundTruth);
    ~VisualOdometry(){}

    void DetectFeature(const cv::Mat& img);
    void ShowFeature(const cv::Mat& img);
    void ExtractFisrt();
    void ExtractSecond();
    void ExtractRest();
    void CalEssential();
    double GetAbsoluteScale(const int &frame_id);
    void FeatureDetection();
    inline cv::Mat getTranslation()
    {
        return mt;
    }

public:

    FrameState mFrameState;
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

    int mCount;
    double mScale;
    bool bUseDataset;
    std::string mstrGroundTruth;
};

} //namespace Light_SLAM
#endif // VISUALODOMETRY_H
