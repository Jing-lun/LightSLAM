#ifndef RUNSLAM_H
#define RUNSLAM_H

#include "VisualOdometry.h"
#include "VO.h"

#include <string>

namespace Light_SLAM
{

class VisualOdometry;
class VO;

class RunSLAM
{

public:
    
    // RunSLAM(VisualOdometry* pVO);
    RunSLAM(VO* pVO);
    ~RunSLAM(){}

    void vo(const cv::Mat& img);

public:

    // VisualOdometry* mpTest1;
    VO* mpTest1;
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