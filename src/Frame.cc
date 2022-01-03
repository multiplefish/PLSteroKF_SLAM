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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>

#include<pthread.h>
namespace ORB_SLAM2
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame()
{}

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2),
     mLdesc(frame.mLdesc),
     mRdesc(frame.mRdesc),  
     NL(frame.NL), 
     mvKeylinesUn(frame.mvKeylinesUn),  
     mvKeylinesRightUn(frame.mvKeylinesRightUn),
     mvpMapLines(frame.mvpMapLines),  //线特征相关的类成员变量
     mvbLineOutlier(frame.mvbLineOutlier), 
     mvKeyLineFunctions(frame.mvKeyLineFunctions),
     mvKeyLineRightFunctions(frame.mvKeyLineRightFunctions),
     mvDepthLineStart(frame.mvDepthLineStart),
     mvDepthLineEnd(frame.mvDepthLineEnd),
     mvuRightLineStart(frame.mvuRightLineStart),
     mvuRightLineEnd(frame.mvuRightLineEnd)

{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}

// 双目
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth
            ,bool _lineways,int _LSDLines)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),LSDLines(_LSDLines),
     mpReferenceKF(static_cast<KeyFrame*>(NULL))
{
    
    mvbLine=_lineways;
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    std::thread threadLeft([&]()
    {
            (*mpORBextractorLeft)(imLeft,cv::Mat(),mvKeys,mDescriptors);
    }
    );

    std::thread threadRight([&]()
    {
        (*mpORBextractorRight)(imRight,cv::Mat(),mvKeysRight,mDescriptorsRight);
    }
    );

    std::thread threadLineLeft([&]()
    {
        mpLineSegment->ExtractLineSegment(imLeft, mvKeylinesUn, mLdesc, mvKeyLineFunctions,LSDLines);
    });

    std::thread threadLineRight([&]()
    {
        mpLineSegment->ExtractLineSegment(imRight, mvKeylinesRightUn, mRdesc, mvKeyLineRightFunctions,LSDLines);
    }
    );
    threadLeft.join();
    threadRight.join();
    threadLineLeft.join();
    threadLineRight.join();
//    thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
//    thread threadRight(&Frame::ExtractORB,this,1,imRight);
//    thread threadLineLeft(&Frame::ExtractLSD,this,0,imLeft);
//    thread threadLineRight(&Frame::ExtractLSD,this,1,imRight);
//
//    threadLeft.join();
//    threadRight.join();
//    threadLineLeft.join();
//    threadLineRight.join();

    N = mvKeys.size();
    NL = mvKeylinesUn.size();

    if(N+NL==0)
        return;

    UndistortKeyPoints();

    // ComputeStereoMatches();
    std::thread ComputeStereoMatchesThread(&Frame::ComputeStereoMatches,this);
    std::thread ComputeStereoMatchesLineThread(&Frame::ComputeStereoMatchesLine,this);
    ComputeStereoMatchesThread.join();
    ComputeStereoMatchesLineThread.join();

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));    
    mvbOutlier = vector<bool>(N,false);
    mvpMapLines = vector<MapLine*>(NL,static_cast<MapLine*>(NULL));   
    mvbLineOutlier = vector<bool>(NL, false);


    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();    
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoFromRGBD(imDepth);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}


Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos(); 

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}


void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}

cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}
void Frame::ExtractLSD(int flag,const cv::Mat &im)
{
        // 判断是左图还是右图

    if(flag==0)
    {  
            mpLineSegment->ExtractLineSegment(im, mvKeylinesUn, mLdesc, mvKeyLineFunctions,LSDLines);
    }
    else
    {
            mpLineSegment->ExtractLineSegment(im, mvKeylinesRightUn, mRdesc, mvKeyLineRightFunctions,LSDLines);
    }
}

bool Frame::isInFrustum(MapLine *pML, float viewingCosLimit)
{
    pML->mbTrackInView = false;

    Vector6d P = pML->GetWorldPos();

    cv::Mat SP = (Mat_<float>(3,1) << P(0), P(1), P(2));
    cv::Mat EP = (Mat_<float>(3,1) << P(3), P(4), P(5));

    // 两个端点在相机坐标系下的坐标
    const cv::Mat SPc = mRcw*SP + mtcw;
    const float &SPcX = SPc.at<float>(0);
    const float &SPcY = SPc.at<float>(1);
    const float &SPcZ = SPc.at<float>(2);

    const cv::Mat EPc = mRcw*EP + mtcw;
    const float &EPcX = EPc.at<float>(0);
    const float &EPcY = EPc.at<float>(1);
    const float &EPcZ = EPc.at<float>(2);

    // 检测两个端点的Z值是否为正
    if(SPcZ<0.0f || EPcZ<0.0f)
        return false;

    // V-D 1) 将端点投影到当前帧上，并判断是否在图像内
    const float invz1 = 1.0f/SPcZ;
    const float u1 = fx * SPcX * invz1 + cx;
    const float v1 = fy * SPcY * invz1 + cy;

    if(u1<mnMinX || u1>mnMaxX)
        return false;
    if(v1<mnMinY || v1>mnMaxY)
        return false;

    const float invz2 = 1.0f/EPcZ;
    const float u2 = fx*EPcX*invz2 + cx;
    const float v2 = fy*EPcY*invz2 + cy;

    if(u2<mnMinX || u2>mnMaxX)
        return false;
    if(v2<mnMinY || v2>mnMaxY)
        return false;

    // V-D 3)计算MapLine到相机中心的距离，并判断是否在尺度变化的距离内
    const float maxDistance = pML->GetMaxDistanceInvariance();
    const float minDistance = pML->GetMinDistanceInvariance();
 

    // if(minDistance<-300||maxDistance>300)
    // {
    // i
    //     minDistance=-300;
    //     maxDistance=300;
    // return false;
    // }

    // 世界坐标系下，相机到线段中点的向量，向量方向由相机指向中点
    const cv::Mat OM = 0.5*(SP+EP) - mOw;
    const float dist = cv::norm(OM);
    if(dist<minDistance || dist>maxDistance)
    {
           return false;
    }
    // Check viewing angle
    // V-D 2)计算当前视角和平均视角夹角的余弦值，若小于cos(60°),即夹角大于60°则返回
    Vector3d Pn = pML->GetNormal();
    cv::Mat pn = (Mat_<float>(3,1) << Pn(0), Pn(1), Pn(2));
    const float viewCos = OM.dot(pn)/dist;
    if(viewCos<viewingCosLimit)
    {
           return false;
    }
        

    // Predict scale in the image
    // V-D 4) 根据深度预测尺度（对应特征在一层）
    const int nPredictedLevel = pML->PredictScale(dist, mfLogScaleFactor);

    // Data used by the tracking
    // 标记该特征将来要被投影
    pML->mbTrackInView = true;
    pML->mTrackProjX1 = u1;
    pML->mTrackProjY1 = v1;
    pML->mTrackProjX2 = u2;
    pML->mTrackProjY2 = v2;
    pML->mTrackProjXRS= u1 - mbf*invz1;    // bf/z其实是视差，相减得到右图（如有）中对应点的横坐标
    pML->mTrackProjXRE= u2 - mbf*invz2; 
    pML->mnTrackScaleLevel = nPredictedLevel;
    pML->mTrackViewCos = viewCos;

    return true;
}

vector<size_t> Frame::GetLinesInArea(const float &x1, const float &y1, const float &x2, const float &y2, const float &r,
                                     const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;

    vector<KeyLine> vkl = this->mvKeylinesUn;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>0);
    
//    for(vector<KeyLine>::iterator vit=vkl.begin(), vend=vkl.end(); vit!=vend; vit++)
    for(size_t i=0; i<vkl.size(); i++)
    {
        KeyLine keyline = vkl[i];

        // 1.对比中点距离
        float distance = (0.5*(x1+x2)-keyline.pt.x)*(0.5*(x1+x2)-keyline.pt.x)+(0.5*(y1+y2)-keyline.pt.y)*(0.5*(y1+y2)-keyline.pt.y);
        if(distance > r*r)
            continue;
      
        // 2.比较斜率，KeyLine的angle就是代表斜率
        float slope = (y1-y2)/(x1-x2)-keyline.angle;
        if(slope > r*0.01)
            continue;
        // 3.比较金字塔层数
        if(bCheckLevels)
        {
            if(keyline.octave<minLevel)
                continue;
            if(maxLevel>=0 && keyline.octave>maxLevel)
                continue;
        }
  
        vIndices.push_back(i);
    }

    return vIndices;
}
Vector6d Frame::obtain3DLine(const int &i)
{

/** <li> 获取这个特征点的深度（这里的深度可能是通过双目视差得出的，也可能是直接通过深度图像的出来的） </li> */
   
    // 防止溢出

    const float zs = mvDepthLineStart[i];
    const float  ze = mvDepthLineEnd[i];
    Vector6d Lines3D;
    if(zs>0&&ze>0)
    {
        const float us = mvKeylinesUn[i].startPointX;
        const float vs = mvKeylinesUn[i].startPointY;

        const float ue = mvKeylinesUn[i].endPointX;
        const float ve= mvKeylinesUn[i].endPointY;
        // drawKeylines();
		//计算在当前相机坐标系下的坐标
        const float xs = (us-cx)*zs*invfx;
        const float ys = (vs-cy)*zs*invfy;

        const float xe= (ue-cx)*ze*invfx;
        const float ye = (ve-cy)*ze*invfy;

        cv::Mat Ac =(Mat_<float>(3,1) <<xs,ys,zs);
        cv::Mat A= mRwc*Ac+mOw;
        cv::Mat Bc = (Mat_<float>(3,1) <<xe,ye,ze);
        cv::Mat B= mRwc*Bc+mOw;

        // float len=sqrt((A.at<float>(0,0)-B.at<float>(0,0))*(A.at<float>(0,0)-B.at<float>(0,0))+
        // (A.at<float>(1,0)-B.at<float>(1,0))*(A.at<float>(1,0)-B.at<float>(1,0))+
        // (A.at<float>(2,0)-B.at<float>(2,0)*(A.at<float>(2,0)-B.at<float>(2,0))));
        // if(len>4.0f)
        // Lines3D<<0,0,0,0,0,0;
        // else
        Lines3D<<A.at<float>(0,0),A.at<float>(1,0),A.at<float>(2,0),B.at<float>(0,0),B.at<float>(1,0),B.at<float>(2,0);
        


         return Lines3D;
        
    }
    else
    {

    Lines3D<<0,0,0,0,0,0;
    return Lines3D;
    }
}
void Frame::ComputeStereoMatchesLine()
{   

    #if true
    mvDepthLineStart = vector<float>(NL,-1.0f);
    mvDepthLineEnd = vector<float>(NL,-1.0f);
    mvuRightLineEnd=vector<float>(NL,-1.0f);
    mvuRightLineStart=vector<float>(NL,-1.0f);
    int nmatches = 0;

    vector<KeyLine>lines_l=mvKeylinesUn;
    vector<KeyLine>lines_r=mvKeylinesRightUn;

    Mat ldesc1=mLdesc; 
    Mat ldesc2=mRdesc;
      if(  !lines_l.empty() && !lines_r.empty() )
      {
    
        vector<vector<DMatch>> lmatches_lr, lmatches_rl;
        Mat ldesc_l_;
         // LR and RL matches
        thread threadLR(&Frame::GetMatchLineFrame,this,ldesc1,ldesc2,0);
        thread threadRL(&Frame::GetMatchLineFrame,this,ldesc1,ldesc2,1);
        threadLR.join();
        threadRL.join();

        lmatches_lr=mvlmatcheslr;
        lmatches_rl=mvlmatchesrl;
        double nn_dist_th, nn12_dist_th;
        //线特征的MAD中位数绝对偏差
        mpLineSegment->lineDescriptorMAD(lmatches_lr,nn_dist_th, nn12_dist_th);        
        nn12_dist_th  = nn12_dist_th * 0.5;
         // sort matches by the distance between the best and second best matches
         // bucle around pmatches
         sort( lmatches_lr.begin(), lmatches_lr.end(), sort_descriptor_by_queryIdx() );
         sort( lmatches_rl.begin(), lmatches_rl.end(), sort_descriptor_by_queryIdx() );
         int n_matches;
         n_matches = min(lmatches_lr.size(),lmatches_rl.size());
         int ls_idx = 0;  
         for( int i = 0; i < n_matches; i++ )
         {
             // check if they are mutual best matches ( if bestLRMatches() )
             int lr_qdx = lmatches_lr[i][0].queryIdx;
             int lr_tdx = lmatches_lr[i][0].trainIdx;
             int rl_tdx;
             rl_tdx = lmatches_rl[lr_tdx][0].trainIdx;
             // check if they are mutual best matches and the minimum distance
             double dist_12 = lmatches_lr[i][1].distance - lmatches_lr[i][0].distance;
             double length  = lines_r[lr_tdx].lineLength;
             if( lr_qdx == rl_tdx && dist_12 > nn12_dist_th )
             {
                 // estimate the disparity of the endpoints
                 // 计算左img线特征重投影后的端点距离
                 Vector3d sp_l; sp_l << lines_l[lr_qdx].startPointX, lines_l[lr_qdx].startPointY, 1.0;
                 Vector3d ep_l; ep_l << lines_l[lr_qdx].endPointX,   lines_l[lr_qdx].endPointY,   1.0;
                 // le_l是sp_l和ep_l的叉乘的单位向量
                 Vector3d le_l; le_l << sp_l.cross(ep_l); le_l = le_l / sqrt( le_l(0)*le_l(0) + le_l(1)*le_l(1));
                 // 计算右img线特征重投影后的端点距离
                 Vector3d sp_r; sp_r << lines_r[lr_tdx].startPointX, lines_r[lr_tdx].startPointY, 1.0;
                 Vector3d ep_r; ep_r << lines_r[lr_tdx].endPointX,   lines_r[lr_tdx].endPointY,   1.0;
                 // le_r是sp_r和ep_r的叉乘向量
                 Vector3d le_r; le_r << sp_r.cross(ep_r);le_r = le_r / sqrt( le_r(0)*le_r(0) + le_r(1)*le_r(1));
                 // 计算左右img的投影线段的重合比率
                 
                 double overlap = mpLineSegment->lineSegmentOverlap( sp_l(1), ep_l(1), sp_r(1), ep_r(1) );
            
                 sp_r << - (le_r(2)+le_r(1)*lines_l[lr_qdx].startPointY )/le_r(0) , lines_l[lr_qdx].startPointY ,  1.0;
                 ep_r << - (le_r(2)+le_r(1)*lines_l[lr_qdx].endPointY   )/le_r(0) , lines_l[lr_qdx].endPointY ,    1.0;
                 double disp_s = lines_l[lr_qdx].startPointX - sp_r(0);
                 double disp_e = lines_l[lr_qdx].endPointX   - ep_r(0);
                // double disp_s, disp_e;
                // sp_r << ( sp_r(0)*( sp_l(1) - ep_r(1) ) + ep_r(0)*( sp_r(1) - sp_l(1) ) ) / ( sp_r(1)-ep_r(1) ) , sp_l(1) ,  1.0;
                // ep_r << ( sp_r(0)*( ep_l(1) - ep_r(1) ) + ep_r(0)*( sp_r(1) - ep_l(1) ) ) / ( sp_r(1)-ep_r(1) ) , ep_l(1) ,  1.0;
                // filterLineSegmentDisparity( sp_l.head(2), ep_l.head(2), sp_r.head(2), ep_r.head(2), disp_s, disp_e );

                if(disp_s<0.1) continue;
                if(disp_e<0.1) continue;
                if(overlap<0.75) continue;
                // if(std::abs( sp_l(1)-ep_l(1) )<0.1) continue;
                // if(std::abs( sp_r(1)-ep_r(1) )<0.1) continue;


                if(mvbLine)
                 {
                    float firstDepth=mbf/disp_s;float secondDepth=mbf/disp_e;
                    if(firstDepth<=mThDepth&&secondDepth<=mThDepth&&firstDepth>=0&&secondDepth>=0)
                    {
                    mvDepthLineStart[i]=firstDepth;
                    mvDepthLineEnd[i]=secondDepth;
                    float us=lines_l[lr_qdx].startPointX-mbf/mvDepthLineStart[i];
                    float ue=lines_l[lr_qdx].endPointX-mbf/mvDepthLineEnd[i];
                    
                    float temp1=- (le_r(2)+le_r(1)*lines_l[lr_qdx].startPointY )/le_r(0);
                    float temp2=- (le_r(2)+le_r(1)*lines_l[lr_qdx].endPointY   )/le_r(0);

                    // cout<<temp1-us<<" "<<temp2-ue<<endl;
                    if(abs(temp1-us)>1) continue;
                    if(abs(temp2-ue)>1) continue;



                    mvuRightLineStart[i]=- (le_r(2)+le_r(1)*lines_l[lr_qdx].startPointY )/le_r(0);
                    mvuRightLineEnd[i]= - (le_r(2)+le_r(1)*lines_l[lr_qdx].endPointY   )/le_r(0);
                    }

                 }
                 else{
                    mvuRightLineStart[i]=- 1;
                    mvuRightLineEnd[i]= - 1;
                    mvDepthLineStart[i]=-1;
                    mvDepthLineEnd[i]=-1;
                 }
             }
         }
    }
    #endif
    #if false
    mvDepthLineStart = vector<float>(NL,-1.0f);
    mvDepthLineEnd = vector<float>(NL,-1.0f);
    mvuRightLineEnd=vector<float>(NL,-1.0f);
    mvuRightLineStart=vector<float>(NL,-1.0f);
    int nmatches = 0;

    vector<KeyLine>lines_l=mvKeylinesUn;
    vector<KeyLine>lines_r=mvKeylinesRightUn;

    Mat ldesc1=mLdesc; 
    Mat ldesc2=mRdesc;
    // BFMatcher.knn

        //Perform a brute force between Keypoint in the left and right image
    vector<vector<DMatch>> Linematches;
    BFMatcher* bfm = new BFMatcher( NORM_HAMMING, false );
    bfm->knnMatch(ldesc1,ldesc2,Linematches,2);

    int nMatches = 0;
    int descMatches = 0;
    
    for(int i=0;i<Linematches.size();++i)
    {
        int lr_qdx = Linematches[i][0].queryIdx;
        int lr_tdx = Linematches[i][0].trainIdx;

        descMatches++;    

        if (Linematches[i].size()>=2&&Linematches[i][0].distance<Linematches[i][1].distance*0.828)
        {
                            //线特征的MAD中位数绝对偏差
            double nn_dist_th=0;
            double nn12_dist_th=0;
            lineDescriptorMAD(Linematches,nn_dist_th, nn12_dist_th);  
            if((Linematches[i][0].distance-Linematches[i][1].distance)>(nn12_dist_th*0.5))
            continue;

            /* 目标主要是检查重投影误差去纠正无匹配点 */
            float sigma1=mvLevelSigma2[lines_l[Linematches[i][0].queryIdx].octave];
            float sigma2=mvLevelSigma2[lines_r[Linematches[i][0].trainIdx].octave];
            // / estimate the disparity of the endpoints
            int lr_qdx = Linematches[i][0].queryIdx;
            int lr_tdx = Linematches[i][0].trainIdx;
                 // 计算左img线特征重投影后的端点距离
            Vector3d sp_l; sp_l << lines_l[lr_qdx].startPointX, lines_l[lr_qdx].startPointY, 1.0;
            Vector3d ep_l; ep_l << lines_l[lr_qdx].endPointX,   lines_l[lr_qdx].endPointY,   1.0;
                 // le_l是sp_l和ep_l的叉乘的单位向量
            Vector3d le_l; le_l << sp_l.cross(ep_l); le_l = le_l / sqrt( le_l(0)*le_l(0) + le_l(1)*le_l(1) );
                 // 计算右img线特征重投影后的端点距离
            Vector3d sp_r; sp_r << lines_r[lr_tdx].startPointX, lines_r[lr_tdx].startPointY, 1.0;
            Vector3d ep_r; ep_r << lines_r[lr_tdx].endPointX,   lines_r[lr_tdx].endPointY,   1.0;
                 // le_r是sp_r和ep_r的叉乘向量
            Vector3d le_r; le_r << sp_r.cross(ep_r); le_r = le_r / sqrt( le_r(0)*le_r(0) + le_r(1)*le_r(1) );
                 // 计算左右img的投影线段的重合比率
            double overlap = lineSegmentOverlapStereo(sp_l(1), ep_l(1), sp_r(1), ep_r(1) );
            if(overlap<0.7) continue;
            // x=by+c/a
            sp_r << - (le_r(2)+le_r(1)*lines_l[lr_qdx].startPointY )/le_r(0) , lines_l[lr_qdx].startPointY ,  1.0;
            ep_r << - (le_r(2)+le_r(1)*lines_l[lr_qdx].endPointY   )/le_r(0) , lines_l[lr_qdx].endPointY ,    1.0;

            double disp_s = lines_l[lr_qdx].startPointX - sp_r(0);
            double disp_e = lines_l[lr_qdx].endPointX   - ep_r(0);
        

            if(disp_s>=0.1&&disp_e>=0.1)
            {
                float firstDepth=mbf/disp_s;float secondDepth=mbf/disp_e;
                if(firstDepth<=mThDepth&&secondDepth<=mThDepth&&firstDepth>=0&&secondDepth>=0)
                {
                    mvDepthLineStart[i]=firstDepth;
                    mvDepthLineEnd[i]=secondDepth;

                    mvuRightLineStart[i]=- (le_r(2)+le_r(1)*lines_l[lr_qdx].startPointY )/le_r(0);
                    mvuRightLineEnd[i]= - (le_r(2)+le_r(1)*lines_l[lr_qdx].endPointY   )/le_r(0);


                }

            }
        }
        


    }
    #endif
}
void  Frame::GetMatchLineFrame(const cv::Mat &L1,const cv::Mat &L2,const int flag)
{
    if(flag==0)
    {
            BFMatcher* bfm = new BFMatcher( NORM_HAMMING, false );
            bfm->knnMatch(L1, L2,mvlmatcheslr, 2);
    }
    else
    {
            BFMatcher* bfm = new BFMatcher( NORM_HAMMING, false );
            bfm->knnMatch(L2, L1,mvlmatchesrl, 2);
    }

}
void Frame::filterLineSegmentDisparity( Vector2d spl, Vector2d epl, Vector2d spr, Vector2d epr, double &disp_s, double &disp_e )
{
    disp_s = spl(0) - spr(0);
    disp_e = epl(0) - epr(0);
    // if they are too different, ignore them
    if(  min( disp_s, disp_e ) / max( disp_s, disp_e ) < 0.7 )
    {
        disp_s = -1.0;
        disp_e = -1.0;
    }
}
// 
} //namespace ORB_SLAM
