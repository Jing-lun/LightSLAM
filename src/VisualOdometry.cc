#include <vector>
#include <iostream>
#include "VisualOdometry.h"

using namespace cv;
using namespace std;

namespace Light_SLAM
{
    VisualOdometry::VisualOdometry(const std::string &strSettingPath):framestate(INIT_FRAME),mMatcher(new flann::LshIndexParams(5,10,2))
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

        int ft_num = fSettings["number_of_features"];
        double pyramid_scale = fSettings["scale_factor"];
        int pyramid_level = fSettings["level_pyramid"];
        mpORB = ORB::create(ft_num, pyramid_scale, pyramid_level);
        // mpMatcher = DescriptorMatcher::create("BruteForce-Hamming");
        // mpMatcher = new flann::LshIndexParams(5,10,2);
    }

    void VisualOdometry::DetectFeature(Mat img)
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
            case LOST_FRAME:
                // TODO(allen.fengjl@gmail.com) add the lost condition
                break;
        }
        mvLastKeyPoints = mvKeyPoints;
        mLastImage = mCurrentImage;
        mLastDesp = mDesp;
    }

    void VisualOdometry::ExtractFisrt()
    {
        mpORB->detectAndCompute(mCurrentImage, Mat(), mvKeyPoints, mDesp); 

        framestate = FrameState::SECOND_FRAME;
    }

    void VisualOdometry::ExtractSecond()
    {
        mpORB->detectAndCompute(mCurrentImage, Mat(), mvKeyPoints, mDesp); 

        mMatcher.match(mLastDesp, mDesp, mvMatches);

        double max_dist = 0; double min_dist = 100;
        // Quick calculation of max and min distances between keypoints
        for( int i = 0; i < mLastDesp.rows; i++ )
        { 
            double dist = mvMatches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }

        // Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
        // or a small arbitary value ( 0.02 ) in the event that min_dist is very
        // small)
        // PS.- radiusMatch can also be used here.
        mvGoodMatches.clear();
        for( int i = 0; i < mLastDesp.rows; i++ )
        { 
            if( mvMatches[i].distance <= max(2*min_dist, 0.02) )
            { 
                mvGoodMatches.push_back( mvMatches[i]);
            }
        }

        cout<<"good matches: "<<mvGoodMatches.size()<<endl;
    }

    void VisualOdometry::ShowFeature(Mat img)
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

    void VisualOdometry::CalEssential()
    {
        vector<Point2f> points1;
        vector<Point2f> points2;
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