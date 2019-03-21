#ifndef RUNSLAM_H
#define RUNSLAM_H

#include "VisualOdometry.h"

#include <string>

namespace Light_SLAM
{

class PoseEstimation2Dto2D;
class ExtractFeature;

class RunSLAM
{

public:
    
    RunSLAM(VisualOdometry* pExtract);
    ~RunSLAM(){}

    void VO(const cv::Mat img);

public:

    VisualOdometry* mpTest1;
    int mFontFace;
    double mFontScale;
    int mFontThickness;
    cv::Point mText;
    cv::Mat mTrajectory;

    std::string mstrFrameName;
    std::string mstrTrajName;
    char text[200];
};

} // Light_SLAM
#endif // RUNSLAM_H