/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>
#include "auxiliar.h"
#include "ExtractLineSegment.h"
#include "MapLine.h"
#include "LSDmatcher.h"

#include<opencv2/core/core.hpp>


using namespace Eigen;
#include<Eigen/Dense>

namespace ORB_SLAM2
{
class Viewer;
class FrameDrawer;
// class LSDextractor;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class PID {
    public:
        float CurMatches;//输入变量，即期望输出的变量值
        float RFcMatches;//实际输出变量，即采样回来的输出变量
        float Err; //误差值
        float Err_last;//上一次误差值
        float Kp, Ki, Kd;//比例 积分 微分系数
        float Res;
    } ;

class KalmanKF
{

    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KalmanKF(){

        U=4e-3;
        P=Matrix6d::Identity();
        Q=Matrix6d::Identity();

        A.block<3,3>(0,0)<<Matrix3d::Identity();
        A.block<3,3>(0,3)<<Matrix3d::Identity();
        A.block<3,3>(3,0)<<Matrix3d::Zero();
        A.block<3,3>(3,3)<<Matrix3d::Identity();

        H.block<3,3>(0,0)=Matrix3d::Identity();
        H.block<3,3>(0,3)=Matrix3d::Zero();
        
        R=Matrix3d::Identity();

        B=Vector6d::Identity();
        X=Vector6d::Zero();

        isfirst=true;

    }

    void init();
    void setvalue();


    public:
    float U ;
    double dt; //

    Eigen::Matrix<double,6,6>  P,Q,A,At;
    Eigen::Matrix<double,3,6> H;
    Eigen::Matrix<double,6,3> Ht,K;
    Eigen::Matrix<double,3,3> R;

    Vector6d B,X;//状态值

    queue<Vector3d>VV;//观察值
    bool isfirst;

    
};
class Tracking
{  

public:
    // friend KalmanKF;
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);
    void initkf();

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);
    float PID_Cal(float CurMatches,float RFMatches);
    template<class T1, class T2,class T3,class T4,class T5>
    void writeTxt(T1 T,T2 R,T3 P,T4 E,T5 Y,string filename);


public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();

    PID pid;
    KalmanKF *KF;

    float LastVelocity;
    float curvelocity;
    float acc;
    bool flagkf=true;
    bool isFrist=true;
    queue<float> VelocityPfit;
   
    bool c1d;
    int mbth ;
    float CurV;
    float actualvelocity;
    bool isWriteFile=true;
    int mes=1;


protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();
    void SearchLocalLines();
    bool KalmanKeyFrame();
    void GetKFvalue(void);
    void setvalue();
    bool Polyfit();
    void UpdateLocalLines();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    LineSegment* mpLSDextractorLeft;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;
    int mnMatchesLineInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;
    int mbLine;
    float mvecurth;
    float mfPIDth;
    int mVth;

    list<MapPoint*> mlpTemporalPoints;
    list<MapLine*>mlpTemporalLines;

    std::vector<MapLine*> mvpLocalMapLines;
    
    int mnLineMatchesInliers;

    int miLSDLines;
    

};

} //namespace ORB_SLAM

#endif // TRACKING_H
