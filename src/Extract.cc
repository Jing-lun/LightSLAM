#include <vector>
#include <iostream>
#include "Extract.h"

using namespace cv;
using namespace std;

namespace Light_SLAM
{
    static int mCount;
    ExtractFeature::ExtractFeature():framestate(INIT_FRAME)
    {
        mpORB = ORB::create();
        mpMatcher = DescriptorMatcher::create("BruteForce-Hamming");
        mCount = 0;
    }

    void ExtractFeature::DetectFeature(Mat img)
    {   
        mCurrentImage = img;
        switch(framestate)
        {
            case INIT_FRAME:
                ExtractFisrt();
                break;
            case SECOND_FRAME:
                ExtractSecond();
                break;
        }
        mvLastKeyPoints = mvKeyPoints;
        mLastImage = mCurrentImage;
        mLastDesp = mDesp;
    }

    void ExtractFeature::ExtractFisrt()
    {
        mpORB->detect(mCurrentImage, mvKeyPoints);
        mpORB->compute(mCurrentImage, mvKeyPoints, mDesp); 

        framestate = FrameState::SECOND_FRAME;
    }

    void ExtractFeature::ExtractSecond()
    {
        mpORB->detect(mCurrentImage, mvKeyPoints);
        mpORB->compute(mCurrentImage, mvKeyPoints, mDesp);

        mpMatcher->match(mLastDesp, mDesp, mvMatches);
        // cout << "find out totall " << mvMatches.size() << " matches" <<endl;
        mvGoodMatches.clear();

        double min_dist = 10000;
        double max_dist = 0;

        double minDis = 9999;
        for ( size_t i=0; i<mvMatches.size(); i++ )
        {
            if ( mvMatches[i].distance < minDis )
                minDis = mvMatches[i].distance;
        }
        cout<<"min dis = "<<minDis<<endl;

        for ( size_t i=0; i<mvMatches.size(); i++ )
        {
            if (mvMatches[i].distance < 10*minDis)
                mvGoodMatches.push_back( mvMatches[i] );
        }
        cout << "find out totall " << mvGoodMatches.size() << " good matches" <<endl;
    }

    void ExtractFeature::ShowFeature(Mat img)
    {
        Mat ShowKeypoints;
        Mat ShowLastKeypoints;
        Mat ShowMatches;

        drawMatches(mLastImage, mvLastKeyPoints, img, mvKeyPoints, mvGoodMatches, ShowMatches);
        imshow("good_matches", ShowMatches);
        imwrite("./good_matches.png", ShowMatches);
        waitKey(0);
        drawMatches(mLastImage, mvLastKeyPoints, img, mvKeyPoints, mvMatches, ShowMatches);
        imshow("matches", ShowMatches);
        imwrite("./matches.png", ShowMatches);
        waitKey(0);
        
        // drawKeypoints(img, mvKeyPoints, ShowKeypoints);
        // drawKeypoints(mLastImage, mvLastKeyPoints, ShowLastKeypoints);
        // imshow("keypoints", ShowKeypoints);
        // imwrite("./keypoints.png", ShowKeypoints);
        // waitKey(0);
    }

} //namespace Light_SLAM