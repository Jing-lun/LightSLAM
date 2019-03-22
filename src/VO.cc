#include <vector>
#include <iostream>
#include "VO.h"

using namespace cv;
using namespace std;

namespace Light_SLAM
{
    VO::VO(const std::string &strSettingPath):mFrameState(INIT_FRAME),
                                            mCount(0), bUseDataset(false)
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
    }

    VO::VO(const std::string &strSettingPath, const std::string &strGroundTruth):
                                    mFrameState(INIT_FRAME), 
                                    mCount(0), bUseDataset(true)
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
    }

    void VO::ProcessFrames(const Mat& img)
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
    }

    void VO::ProcessFirstFrame()
    {
        FeatureExtraction();
        GetGroundTruth();
        mFrameState = FrameState::SECOND_FRAME;
    }

    void VO::ProcessSecondFrame()
    {
        FeatureTracking();
        Mat E, R, T;
        
        E = findEssentialMat(mvCurrentPoints, mvLastPoints, mFocalLength, mOpticalCenter);
        recoverPose(E, mvCurrentPoints, mvLastPoints, R, T, mFocalLength, mOpticalCenter);
        R.copyTo(mR);
        T.copyTo(mT);
        mFrameState = FrameState::REST_FRAME;

        mvLastPoints = mvCurrentPoints;
        GetGroundTruth();
    }

    void VO::ProcessRestFrames()
    {
        FeatureTracking();
        cv::Mat E, R, T;
        E = findEssentialMat(mvCurrentPoints, mvLastPoints, mFocalLength, mOpticalCenter);
        recoverPose(E, mvCurrentPoints, mvLastPoints, R, T, mFocalLength, mOpticalCenter);
        mScale = getAbsoluteScale(frame_id);
        if(bUseDataset)
        {

        cout<<mScale<<endl;
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
        cout<<"mT = "<<endl<<mT<<endl;
        cout<<"mR = "<<endl<<mR<<endl;
        if(mvLastKeyPoints.size() < 2000)
        {
            FeatureExtraction();
            FeatureTracking();
        }
        else
        {
            mvLastPoints = mvCurrentPoints;
        }
    }

    void VO::GetGroundTruth()
    {
        std::string line;
        getline(mfGroundTruthStream, line);
        std::stringstream ss;
        ss << line;
        double r1, r2, r3, r4, r5, r6, r7, r8, r9;
        ss >> r1 >> r2 >> r3 >> x_pre_ >> r4 >> r5 >> r6 >> y_pre_ >> r7 >> r8 >> r9 >> z_pre_;
    }

    double VO::GetAbsoluteScale()
    {
        std::string line;

        if (std::getline(this->mfGroundTruthStream, line))
        {
            double r1, r2, r3, r4, r5, r6, r7, r8, r9;
            double x = 0, y = 0, z = 0;
            std::istringstream in(line);
            in >> r1 >> r2 >> r3 >> x >> r4 >> r5 >> r6 >> y >> r7 >> r8 >> r9 >> z;
            double scale = sqrt((x - x_pre_) * (x - x_pre_) + (y - y_pre_) * (y - y_pre_) + (z - z_pre_) * (z - z_pre_));
            x_pre_ = x;
            y_pre_ = y;
            z_pre_ = z;
            return scale;
        } 
        else
        {
            return 0;
        }
    }

    void VO::FeatureExtraction()
    {
        int fast_threshold = 20;
        bool non_max_suppression = true;
        FAST(mCurrentImage, mvKeyPoints, fast_threshold, non_max_suppression);
        KeyPoint::convert(mvKeyPoints, mvCurrentPoints);
    }

    void VO::FeatureTracking()
    {
        const double klt_win_size = 21.0;
        const int klt_max_iter = 30;
        const double kly_eps = 0.001;

        vector<uchar> status;
        vector<float> error;
        vector<float> min_eig_vec;
        TermCriteria criteria(TermCriteria::COUNT + TermCriteria::EPS, klt_max_iter, kly_eps);
        calcOpticalFlowPyrLK(mLastImage, mCurrentImage,
                            mvCurrentPoints, mvLastPoints,
                            status, error,
                            cv::Size2i(klt_win_size, klt_win_size),
                            4, criteria, 0);
        disparities_.clear();
        disparities_.reserve(mvLastPoints.size());

        vector<Point2f>::iterator px_prev_it = mvCurrentPoints.begin();
        vector<Point2f>::iterator px_curr_it = mvLastPoints.begin();
        for (size_t i = 0; px_prev_it != mvCurrentPoints.end(); ++i)
        {
            if (!status[i])
            {
                px_curr_it = mvLastPoints.erase(px_curr_it);
                px_prev_it = mvCurrentPoints.erase(px_prev_it);
                continue;
            }
            disparities_.push_back(cv::norm(cv::Point2d(px_prev_it->x - px_curr_it->x, px_prev_it->y - px_curr_it->y)));
            ++px_curr_it;
            ++px_prev_it;
        }
    }

    bool VO::SetGroundTruth()
    {
        this->mfGroundTruthStream.open(mstrGroundTruth);
        if (!this->mfGroundTruthStream.is_open())
        {
            return false;
        } else
        {
            return true;
        }
    }

    double VO::getAbsoluteScale(int frame_id)
    {
        std::string line;
        int i = 0;
        std::ifstream ground_truth(mstrGroundTruth);
        double x = 0, y = 0, z = 0;
        double x_prev, y_prev, z_prev;
        // 获取当前帧真实位置与前一帧的真实位置的距离作为尺度值
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

} // namespace Light_SLAM    