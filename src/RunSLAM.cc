#include "RunSLAM.h"
#include <iostream>
using namespace std;

namespace Light_SLAM
{
    RunSLAM::RunSLAM(ExtractFeature* pExtract, PoseEstimation2Dto2D Estimation):mpTest1(pExtract), mTest2(Estimation)
    {

    }

    void RunSLAM::VO(const cv::Mat img)
    {   
        cv::Mat R;
        cv::Mat t;
        
        if(mpTest1->framestate)
        {
            cout<<mpTest1->framestate<<endl;
            mpTest1->DetectFeature(img);
            try
            {
                mTest2.CalculateEssential(mpTest1->mvLastKeyPoints, mpTest1->mvKeyPoints, 
                                        mpTest1->mvGoodMatches, R, t);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
            
            
        }
        else
        {
            cout<<mpTest1->framestate<<endl;
            mpTest1->DetectFeature(img);
        }
        
    }
}