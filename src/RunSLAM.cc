#include "RunSLAM.h"
#include <iostream>
using namespace std;

namespace Light_SLAM
{
    RunSLAM::RunSLAM(VisualOdometry* pVO):mpTest1(pVO)
    {

    }

    void RunSLAM::VO(const cv::Mat img)
    {   
        if(mpTest1->framestate)
        {
            mpTest1->DetectFeature(img);            
            try
            {
                // mpTest1->ShowFeature(img);
                mpTest1->CalEssential();
            }
            catch(const exception& e)
            {
                cerr << e.what() << '\n';
            }               
        }
        else
        {
            mpTest1->DetectFeature(img);
        }        
    }
}