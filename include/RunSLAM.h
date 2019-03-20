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

    VisualOdometry* mpTest1;
};

} // Light_SLAM
#endif // RUNSLAM_H