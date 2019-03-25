#ifndef VO_H
#define VO_H

#include <vector>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <fstream>

extern int frame_id;
namespace Light_SLAM
{

class VO
{

public:

    enum FrameState
    {
        INIT_FRAME,
        SECOND_FRAME,
        REST_FRAME,
        LOST_FRAME
    };

    VO(const std::string &strSettingPath);
    VO(const std::string &strSettingPath, const std::string &strGroundTruth);
    ~VO(){}

    void ProcessFrames(const cv::Mat& img);
    void ProcessFirstFrame();
    void ProcessSecondFrame();
    void ProcessRestFrames();
    void FeatureExtraction();
    void FeatureTracking();
    double GetAbsoluteScale();
    double getAbsoluteScale(int frame_id);
    void GetGroundTruth();
    bool SetGroundTruth();
    void ShowFeature(const cv::Mat& img);
    inline cv::Mat getTranslation()
    {
        return mT;
    }

public:

    FrameState mFrameState;
    std::vector<cv::KeyPoint> mvKeyPoints;        
    std::vector<cv::KeyPoint> mvLastKeyPoints;
    std::vector<cv::Point2f> mvCurrentPoints;
    std::vector<cv::Point2f> mvLastPoints;
    std::vector<cv::DMatch> mvMatches;
    std::vector<cv::DMatch> mvGoodMatches;

    cv::Mat mLastImage;
    cv::Mat mCurrentImage;

    cv::Mat mK;
    cv::Point2d mOpticalCenter;
    double mFocalLength;
    cv::Mat mR;
    cv::Mat mT;

    int mCount;
    double mScale;
    bool bUseDataset;
    std::string mstrGroundTruth;
    std::ifstream mfGroundTruthStream;

    double x_pre_, y_pre_, z_pre_;
};

} //namespace Light_SLAM
#endif // VO_H
