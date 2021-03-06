#include <vector>
#include <iostream>
#include <fstream>
#include "VisualOdometry.h"

using namespace cv;
using namespace std;
// int frame_id = 0;
int iiii = 0;
namespace Light_SLAM
{
    VisualOdometry::VisualOdometry(const std::string &strSettingPath, const std::string &strGroundTruth):
                                    mstrGroundTruth(strGroundTruth), mstrSettingPath(strSettingPath),
                                    mFrameState(INIT_FRAME), detector(ORB::create(8000)), descriptor(ORB::create(8000)),
                                    mpMatcher(DescriptorMatcher::create ( "BruteForce-Hamming" )), bUseDataset(true)
    {
        cv::FileStorage fSettings(mstrSettingPath, cv::FileStorage::READ); //Read Setting.yaml
        //     |fx  0   cx|
	    // K = |0   fy  cy|
	    //     |0   0   1 |
	    float fx = fSettings["Camera2.fx"];
	    float fy = fSettings["Camera2.fy"];
	    float cx = fSettings["Camera2.cx"];
	    float cy = fSettings["Camera2.cy"];
	    Mat K = cv::Mat::eye(3,3,CV_32F); // initialized as diagnal matrix
	    K.at<float>(0,0) = fx;
	    K.at<float>(1,1) = fy;
	    K.at<float>(0,2) = cx;
	    K.at<float>(1,2) = cy;
	    K.copyTo(mK); 
        mOpticalCenter = cv::Point2d(cx, cy);
        mFocalLength = static_cast<double>(fx);
    }

    void VisualOdometry::ProcessFrames(const Mat& img)
    {
        mCurrentImage = img;
        switch(mFrameState)
        {
            case INIT_FRAME:
                ProcessFirstFrame();
                break;
            case SECOND_FRAME:               
                try
                {
                    ProcessSecondFrame(); 
                }
                catch(const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                }            
                break;
            case REST_FRAME:
                try
                {
                    ProcessRestFrames();
                }
                catch(const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                }
                break;
            case LOST_FRAME:
                cerr << "VO Lost!!" << endl;
                break;
        }
        mLastImage = mCurrentImage;
        mLastDescriptor.release();
        mvLastKeyPoints.clear();
        mLastDescriptor = mDescriptor;
        mvLastKeyPoints = mvKeyPoints;
        mDescriptor.release();
        mvKeyPoints.clear();
    }

    void VisualOdometry::ProcessFirstFrame()
    {
        FeatureExtract();
        mFrameState = FrameState::SECOND_FRAME;
    }

    void VisualOdometry::ProcessSecondFrame()
    {
        FeatureExtract();
        FeatureMatching();

        vector<Point2f> points1;
        vector<Point2f> points2; 

        for ( int i = 0; i < ( int ) mvGoodMatches.size(); i++ )
        {
            points1.push_back ( mvLastKeyPoints[mvGoodMatches[i].queryIdx].pt );
            points2.push_back ( mvKeyPoints[mvGoodMatches[i].trainIdx].pt );
        }

        // FeatureTracking(points1, points2);

        Mat E, R, T, mask;
        
        E = findEssentialMat(points2, points1, mFocalLength, mOpticalCenter, RANSAC, 0.999, 1.0, mask);
        recoverPose(E, points2, points1, R, T, mFocalLength, mOpticalCenter);
        R.copyTo(mR);
        T.copyTo(mT);
        mFrameState = FrameState::REST_FRAME;
    }

    void VisualOdometry::ProcessRestFrames()
    {
        FeatureExtract();
        FeatureMatching();

        vector<Point2f> points1;
        vector<Point2f> points2; 

        for ( int i = 0; i < ( int ) mvGoodMatches.size(); i++ )
        {
            points1.push_back ( mvLastKeyPoints[mvGoodMatches[i].queryIdx].pt );
            points2.push_back ( mvKeyPoints[mvGoodMatches[i].trainIdx].pt );
        }

        // FeatureTracking(points1, points2);

        Mat E, R, T, mask;
        
        E = findEssentialMat(points2, points1, mFocalLength, mOpticalCenter, RANSAC, 0.999, 1.0, mask);
        recoverPose(E, points2, points1, R, T, mFocalLength, mOpticalCenter);
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
        if(mvLastKeyPoints.size() < 1000)
        {
            FeatureExtract();
            FeatureMatching();
        }
    }

    void VisualOdometry::FeatureExtract()
    {
        detector->detect ( mCurrentImage, mvKeyPoints );
        descriptor->compute ( mCurrentImage, mvKeyPoints, mDescriptor );
    }

    void VisualOdometry::FeatureMatching()
    {
        mpMatcher->match ( mLastDescriptor, mDescriptor, mvMatches );

        double min_dist=10000, max_dist=0;
        
        for ( int i = 0; i < mLastDescriptor.rows; i++ )
        {
            double dist = mvMatches[i].distance;
            if ( dist < min_dist ) min_dist = dist;
            if ( dist > max_dist ) max_dist = dist; 
        }
        
        min_dist = min_element( mvMatches.begin(), mvMatches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;
        max_dist = max_element( mvMatches.begin(), mvMatches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;

        mvGoodMatches.clear();
    
        for ( int i = 0; i < mLastDescriptor.rows; i++ )
        {
            if ( mvMatches[i].distance <= max ( 2*min_dist, 30.0 ) )
            {
                mvGoodMatches.push_back ( mvMatches[i] );
            }
        }
    }

    void VisualOdometry::FeatureTracking(const vector<Point2f>& points1, const vector<Point2f>& points2)
    {
        const double klt_win_size = 21.0;
        const int klt_max_iter = 30;
        const double kly_eps = 0.001;

        vector<uchar> status;
        vector<float> error;
        vector<float> min_eig_vec;
        TermCriteria criteria(TermCriteria::COUNT + TermCriteria::EPS, klt_max_iter, kly_eps);
        calcOpticalFlowPyrLK(mLastImage, mCurrentImage,
                            points1, points2,
                            status, error,
                            cv::Size2i(klt_win_size, klt_win_size),
                            4, criteria, 0);
    }

    double VisualOdometry::GetAbsoluteScale(int frame_id)
    {
        string line;
        int i = 0;
        ifstream ground_truth(mstrGroundTruth);
        double x = 0, y = 0, z = 0;
        double x_prev = 0, y_prev = 0, z_prev = 0;
        if (ground_truth.is_open())
        {
            while ((getline(ground_truth, line)) && (i <= frame_id))
            {
                z_prev = z;
                x_prev = x;
                y_prev = y;
                istringstream in(line);
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
            cerr<< "Unable to open file";
            return 0;
        }

        return sqrt((x - x_prev)*(x - x_prev) + (y - y_prev)*(y - y_prev) + (z - z_prev)*(z - z_prev));
    }

} //namespace Light_SLAM