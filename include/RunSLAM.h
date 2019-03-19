#ifndef RUNSLAM_H
#define RUNSLAM_H

#include "PoseEstimation.h"
#include "Extract.h"

#include <string>

namespace Light_SLAM
{

class PoseEstimation2Dto2D;
class ExtractFeature;

class RunSLAM
{

public:
    
    RunSLAM(ExtractFeature* pExtract, PoseEstimation2Dto2D Estimation);
    ~RunSLAM(){}

    void VO(const cv::Mat img);

    std::string mstrSettingPath;
    ExtractFeature* mpTest1;
    PoseEstimation2Dto2D mTest2;
};

} // Light_SLAM
#endif // RUNSLAM_H