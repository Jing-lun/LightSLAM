#include <vector>
#include <iostream>
#include "Extract.h"

using namespace cv;
using namespace std;

namespace Light_SLAM
{
    static int mCount;
    ExtractFeature::ExtractFeature(const std::string &strSettingPath):framestate(INIT_FRAME)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ); //Read Setting.yaml
        //     |fx  0   cx|
	    // K = |0   fy  cy|
	    //     |0   0   1 |
	    float fx = fSettings["Camera.fx"];
	    float fy = fSettings["Camera.fy"];
	    float cx = fSettings["Camera.cx"];
	    float cy = fSettings["Camera.cy"];
	    cv::Mat K = cv::Mat::eye(3,3,CV_32F);// initialized as diagnal matrix
	    K.at<float>(0,0) = fx;
	    K.at<float>(1,1) = fy;
	    K.at<float>(0,2) = cx;
	    K.at<float>(1,2) = cy;
	    K.copyTo(mK); 
        mOpticalCenter = cv::Point2d(cx, cy);
        mFocalLength = static_cast<double>(fx);

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

        float min_dis = std::min_element (
                        mvMatches.begin(), mvMatches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
        {
            return m1.distance < m2.distance;
        } )->distance;

        mvGoodMatches.clear();

        for ( cv::DMatch& m : mvMatches )
        {
            if ( m.distance < max<float> ( min_dis*2.0, 30.0 ) )
            {
                mvGoodMatches.push_back(m);
            }
        }
        cout<<"good matches: "<<mvGoodMatches.size()<<endl;
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
        // drawMatches(mLastImage, mvLastKeyPoints, img, mvKeyPoints, mvMatches, ShowMatches);
        // imshow("matches", ShowMatches);
        // imwrite("./matches.png", ShowMatches);
        // waitKey(0);
        
        // drawKeypoints(img, mvKeyPoints, ShowKeypoints);
        // drawKeypoints(mLastImage, mvLastKeyPoints, ShowLastKeypoints);
        // imshow("keypoints", ShowKeypoints);
        // imwrite("./keypoints.png", ShowKeypoints);
        // waitKey(0);
    }

    void ExtractFeature::CalEssential()
    {
        vector<Point2f> points1;
        vector<Point2f> points2;
        // Point2d mOpticalCenter = Point2d(720.639222, 408.736501);
        // double mFocalLength = 1405.936005;
        Mat mask, E;
        cv::Mat R;
        cv::Mat t;

        for ( int i = 0; i < ( int ) mvGoodMatches.size(); i++ )
        {
            points1.push_back ( mvKeyPoints[mvGoodMatches[i].queryIdx].pt );
            points2.push_back ( mvLastKeyPoints[mvGoodMatches[i].trainIdx].pt );
        }

        // cv::Mat F = cv::findFundamentalMat(points1, points2, CV_FM_8POINT);
        // std::cout << "Fundamental matrix is " << std::endl << F << std::endl;

        E = findEssentialMat(points1, points2, mFocalLength, mOpticalCenter, RANSAC, 0.999, 1.0, mask);
        cout << "Essential matrix is " << endl << E << endl;
        
        recoverPose(E, points1, points2, R, t, mFocalLength, mOpticalCenter, mask);   
        cout << "Rotation matrix is " << endl << R << endl;
        cout << "transform matrix is " << endl << t << endl;
    }

} //namespace Light_SLAM