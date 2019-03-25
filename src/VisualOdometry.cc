#include <vector>
#include <iostream>
#include <fstream>
#include "VisualOdometry.h"

using namespace cv;
using namespace std;
// int frame_id = 0;
namespace Light_SLAM
{
    VisualOdometry::VisualOdometry(const std::string &strSettingPath):mFrameState(INIT_FRAME),
                                    mMatcher(new flann::LshIndexParams(5,10,2)), bUseDataset(false)
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
    }

    VisualOdometry::VisualOdometry(const std::string &strSettingPath, const std::string &strGroundTruth):
                                    mFrameState(INIT_FRAME), mMatcher(new flann::LshIndexParams(5,10,2)), 
                                    bUseDataset(true)
    {
        mstrGroundTruth = strGroundTruth;
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ); //Read Setting.yaml
        //     |fx  0   cx|
	    // K = |0   fy  cy|
	    //     |0   0   1 |
	    float fx = fSettings["Camera2.fx"];
	    float fy = fSettings["Camera2.fy"];
	    float cx = fSettings["Camera2.cx"];
	    float cy = fSettings["Camera2.cy"];
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
    }

    void VisualOdometry::DetectFeature(const Mat& img)
    {   
        mCurrentImage = img;
        switch(mFrameState)
        {
            case INIT_FRAME:
                ExtractFisrt();
                break;
            case SECOND_FRAME:                                
                try
                {
                    ExtractSecond();
                }
                catch(const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                }            
                break;
            case REST_FRAME:
                try
                {
                    ExtractRest();
                }
                catch(const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                }
                break;
            case LOST_FRAME:
                // TODO(allen.fengjl@gmail.com) add the lost condition
                cerr << "VO Lost!!" << endl;
                break;
        }
        mvLastKeyPoints = mvKeyPoints;
        mLastImage = mCurrentImage;
        mLastDesp = mDesp;
        mvKeyPoints.clear();
        mCurrentImage.release();
        mDesp.release();
    }

    void VisualOdometry::ExtractFisrt()
    {
        FeatureExtraction();
        mFrameState = FrameState::SECOND_FRAME;
        // mvLastPoints = mvCurrentPoints;
    }

    void VisualOdometry::ExtractSecond()
    {
        FeatureExtraction();
        FeatureTracking();
        Mat E, R, T, mask;
        
        E = findEssentialMat(mvLastPoints, mvCurrentPoints, mFocalLength, mOpticalCenter, RANSAC, 0.999, 1.0, mask);
        recoverPose(E, mvLastPoints, mvCurrentPoints, R, T, mFocalLength, mOpticalCenter);
        R.copyTo(mR);
        T.copyTo(mT);
        cout<<E<<endl;
        cout<<R<<endl;
        cout<<T<<endl;
        mFrameState = FrameState::REST_FRAME;
        
    }

    void VisualOdometry::ExtractRest()
    {
        FeatureExtraction();
        FeatureTracking();
        cv::Mat E, R, T, mask;
        E = findEssentialMat(mvLastPoints, mvCurrentPoints, mFocalLength, mOpticalCenter, RANSAC, 0.999, 1.0, mask);
        recoverPose(E, mvLastPoints, mvCurrentPoints, R, T, mFocalLength, mOpticalCenter);
        mScale = GetAbsoluteScale(frame_id);
        if(bUseDataset)
        {
            if(mScale > 0.1)
            {
                mT = mT + mScale * (mR * T);
                mR = R * mR; 
            }
        } 
        else
        {
            mT = mT + mR * T;
            mR = R * mR;            
        }
        if(mvLastKeyPoints.size() < 20)
        {
            FeatureExtraction();
            FeatureTracking();
        }
        else
        {
            // mvLastPoints = mvCurrentPoints;
        }
    }

    void VisualOdometry::ShowFeature(const Mat& img)
    {
        Mat ShowKeypoints;
        Mat ShowLastKeypoints;
        Mat ShowMatches;

        drawMatches(mLastImage, mvLastKeyPoints, img, mvKeyPoints, mvGoodMatches, ShowMatches);
        imshow("good_matches", ShowMatches);
        imwrite("good_matches.png", ShowMatches);
        // waitKey(0);
        drawMatches(mLastImage, mvLastKeyPoints, img, mvKeyPoints, mvMatches, ShowMatches);
        imshow("matches", ShowMatches);
        imwrite("matches.png", ShowMatches);
        // waitKey(0);
        
        // drawKeypoints(img, mvKeyPoints, ShowKeypoints);
        // drawKeypoints(mLastImage, mvLastKeyPoints, ShowLastKeypoints);
        // imshow("keypoints", ShowKeypoints);
        // imwrite("./keypoints.png", ShowKeypoints);
        // waitKey(0);
    }

    void VisualOdometry::FeatureExtraction()
    {
        mpORB->detectAndCompute(mCurrentImage, Mat(), mvKeyPoints, mDesp);
        KeyPoint::convert(mvKeyPoints, mvCurrentPoints);
    }

    void VisualOdometry::FeatureTracking()
    {
        // Flann Matcher Method    
        mMatcher.match(mLastDesp, mDesp, mvMatches); //Matching descriptor vectors using FLANN matcher
        cout << "find out totall " << mvMatches.size() << " matches" <<endl;
        
        double min_dist = 10000;
        double max_dist = 0;
        for (int i = 0; i < mLastDesp.rows; ++i)
        {
            double dist = mvMatches[i].distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }
        //Draw only "good" matches (i.e. whose distance is less than 2*min_dist, or a small arbitary value ( 0.02 ) in the event that min_dist is very small)
        mvGoodMatches.clear();
        mvCurrentPoints.clear();
        mvLastPoints.clear();
        for (int i = 0; i < mLastDesp.rows; ++i)
        {
            if (mvMatches[i].distance <= cv::max(2 * min_dist, 30.0))
            {
                mvGoodMatches.push_back(mvMatches[i]);
            }
        }
        cout << "find out totall " << mvGoodMatches.size() << " good matches" <<endl;

        for ( int i = 0; i < ( int ) mvGoodMatches.size(); i++ )
        {
            mvLastPoints.push_back ( mvLastKeyPoints[mvGoodMatches[i].queryIdx].pt );
            mvCurrentPoints.push_back ( mvKeyPoints[mvGoodMatches[i].trainIdx].pt );
        } 
    }

    double VisualOdometry::GetAbsoluteScale(const int& frame_id)
    {
        std::string line;
        int i = 0;
        std::ifstream ground_truth(mstrGroundTruth);
        double x = 0, y = 0, z = 0;
        double x_prev = 0, y_prev = 0, z_prev = 0;
        if (ground_truth.is_open())
        {
            while ((std::getline(ground_truth, line)) && (i <= frame_id))
            {
                z_prev = z;
                x_prev = x;
                y_prev = y;
                std::istringstream in(line);
                for (int j = 0; j < 12; j++)  {
                    in >> z;
                    if (j == 7) y = z;
                    if (j == 3)  x = z;
                }
                i++;
            }
            ground_truth.close();
        }

        else {
            std::cerr<< "Unable to open file";
            return 0;
        }

        return sqrt((x - x_prev)*(x - x_prev) + (y - y_prev)*(y - y_prev) + (z - z_prev)*(z - z_prev));
    }


} //namespace Light_SLAM