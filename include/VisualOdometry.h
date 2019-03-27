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
        REST_FRAME,
        LOST_FRAME
    };

    VisualOdometry(const std::string &strSettingPath, const std::string &strGroundTruth);
    ~VisualOdometry(){}
    void ProcessFrames(const cv::Mat& img);
    void FeatureExtract();
    void FeatureMatching();
    void FeatureTracking(const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2);
    void ProcessFirstFrame();
    void ProcessSecondFrame();
    void ProcessRestFrames();
    double GetAbsoluteScale(int frame_id);
    void ShowTrajectory();
    inline cv::Mat getTranslation()
    {
        return mT;
    }


public:
    std::string mstrGroundTruth;
    std::string mstrSettingPath;
    cv::Mat mK;
    cv::Point2d mOpticalCenter;
    double mFocalLength;
    FrameState mFrameState;
    bool bUseDataset;
    double mScale;

    cv::Mat mCurrentImage;
    cv::Mat mDescriptor;
    std::vector<cv::KeyPoint> mvKeyPoints;
    std::vector<cv::Point2f> mvCurrentPoints;

    cv::Mat mLastImage;
    cv::Mat mLastDescriptor;
    std::vector<cv::KeyPoint> mvLastKeyPoints;
    std::vector<cv::Point2f> mvLastPoints;

    std::vector<cv::DMatch> mvMatches;
    cv::Ptr<cv::DescriptorMatcher> mpMatcher;
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> descriptor;
    std::vector<cv::DMatch> mvGoodMatches;

    cv::Mat mT, mR;       
};

} //namespace Light_SLAM
#endif // VISUALODOMETRY_H
