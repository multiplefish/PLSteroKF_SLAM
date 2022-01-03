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

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include <unistd.h>
#include<mutex>

#include<pthread.h>
namespace ORB_SLAM2
{

LocalMapping::LocalMapping(Map *pMap, const float bMonocular):
    mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{
}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LocalMapping::Run()
{

    mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();
            std::thread mapPointCulling([&]()
            {
                 // Check Recent Added MapPoints
                list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
                const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

                int nThObs;
                if(mbMonocular)
                    nThObs = 2;
                else
                    nThObs = 3;
                const int cnThObs = nThObs;

                while(lit!=mlpRecentAddedMapPoints.end())
                {
                    MapPoint* pMP = *lit;
                    if(pMP->isBad())
                    {
                        lit = mlpRecentAddedMapPoints.erase(lit);
                    }
                    else if(pMP->GetFoundRatio()<0.25f )
                    {
                        pMP->SetBadFlag();
                        lit = mlpRecentAddedMapPoints.erase(lit);
                    }
                    else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
                    {
                        pMP->SetBadFlag();
                        lit = mlpRecentAddedMapPoints.erase(lit);
                    }
                    else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
                        lit = mlpRecentAddedMapPoints.erase(lit);
                    else
                        lit++;
                }
             
             }
            );
   
            std::thread mapLineCulling([&]()
            {
                                // // Check Recent Added MapLines
                // // 获取所有的地图线特征
                 list<MapLine *>::iterator lit = mlpRecentAddedMapLines.begin();
                 // 获取当前的关键帧id
                 const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

                 int nThObs;

                 const int cnThObs = 1;
                  // 遍历等待检查的MapLines
                 while (lit != mlpRecentAddedMapLines.end())
                 {
                     MapLine *pML = *lit;
                     // step1: 将已经是坏的MapLine从检查链中删除
                     if (pML->isBad()) {
                         lit = mlpRecentAddedMapLines.erase(lit);
                     }
                     // 跟踪到该MapPoint的Frame数相比预计可观测到该MapPoint的Frame数的比例需大于25%
                      else if (pML->GetFoundRatio() < 0.02f)
                      {
                          // 地图的线帧数少于25就删除
                          pML->SetBadFlag();
                          lit = mlpRecentAddedMapLines.erase(lit);
                      }
                     //    从该点建立开始，到现在已经过了不小于2个关键帧，但是观测到该点的关键帧数却不超过cnThObs帧，那么该点检验不合格。SetBadFlag()
                     else if (((int) nCurrentKFid - (int) pML->mnFirstKFid) >= 2 && pML->Observations() <= 1)
                     {
                         // 关键帧有了两帧且观察点少于阈值就删除
                         pML->SetBadFlag();
                         lit = mlpRecentAddedMapLines.erase(lit);
                     }
                     // 从建立该点开始，已经过了3个关键帧而没有被剔除，则认为是质量高的点，因此没有
                     else if (((int) nCurrentKFid - (int) pML->mnFirstKFid) >= 3)
                         lit = mlpRecentAddedMapLines.erase(lit);
                     else
                         lit++;
                 }
                    
             }
            );
            mapPointCulling.join();
            mapLineCulling.join();
            
            // Check recent MapPoints

            // Triangulate new MapPoints
            thread threadCreateP(&LocalMapping::CreateNewMapPoints, this);
            thread threadCreateL(&LocalMapping::CreateNewMapLines, this);

            threadCreateP.join();
            threadCreateL.join();

            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                SearchInNeighbors();
            }

            mbAbortBA = false;

            if(!CheckNewKeyFrames() && !stopRequested())
            {
                // Local BA
                 if(mpMap->KeyFramesInMap()>2)
                     Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpMap);

                // Check redundant local Keyframes
                KeyFrameCulling();
            }

            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
        }
        else if(Stop())
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            if(CheckFinish())
                break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(3000);
    }

    SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}


bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                {
                    pMP->AddObservation(mpCurrentKeyFrame, i);
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by the Tracking
                {
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }    

 /// 跟踪局部地图过程中新匹配上的MapLines,和当前关键帧进行绑定
        const vector<MapLine *> vpMapLineMatches = mpCurrentKeyFrame->GetMapLineMatches();
        for (size_t i = 0; i < vpMapLineMatches.size(); i++) {
            MapLine *pML = vpMapLineMatches[i];
            if (pML) {
                if (!pML->isBad()) {
                    if (!pML->IsInKeyFrame(mpCurrentKeyFrame)) {
                        pML->AddObservation(mpCurrentKeyFrame, i);
                        pML->UpdateAverageDir();
                        pML->ComputeDistinctiveDescriptors();
                    } else {
                        mlpRecentAddedMapLines.push_back(pML);
                    }
                }
            }
        }
    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    mpMap->AddKeyFrame(mpCurrentKeyFrame);
}

void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;
        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f )
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}
void LocalMapping::MapLineCulling() 
{
        // // Check Recent Added MapLines
        // 获取所有的地图线特征
        list<MapLine *>::iterator lit = mlpRecentAddedMapLines.begin();
        // 获取当前的关键帧id
        const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

        int nThObs;

        const int cnThObs = 1;
  // 遍历等待检查的MapLines
        while (lit != mlpRecentAddedMapLines.end()) 
        {
            MapLine *pML = *lit;
             // step1: 将已经是坏的MapLine从检查链中删除
            if (pML->isBad()) {
                lit = mlpRecentAddedMapLines.erase(lit);
            } 
            // 跟踪到该MapPoint的Frame数相比预计可观测到该MapPoint的Frame数的比例需大于25%
            else if (pML->GetFoundRatio() < 0.02f)
            {
                // 地图的线帧数少于25就删除
                pML->SetBadFlag();
                lit = mlpRecentAddedMapLines.erase(lit);
            } 
        //    从该点建立开始，到现在已经过了不小于2个关键帧，但是观测到该点的关键帧数却不超过cnThObs帧，那么该点检验不合格。SetBadFlag()
            else if (((int) nCurrentKFid - (int) pML->mnFirstKFid) >= 2 && pML->Observations() <= 1) 
            {
                // 关键帧有了两帧且观察点少于阈值就删除
                pML->SetBadFlag();
                lit = mlpRecentAddedMapLines.erase(lit);
            } 
            // 从建立该点开始，已经过了3个关键帧而没有被剔除，则认为是质量高的点，因此没有
            else if (((int) nCurrentKFid - (int) pML->mnFirstKFid) >= 3)
                lit = mlpRecentAddedMapLines.erase(lit);
            else
                lit++;
        }
}
void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 10;
    if(mbMonocular)
        nn=20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    ORBmatcher matcher(0.6,false);

    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

    int nnew=0;

    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())
            return;

        KeyFrame* pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);

        if(!mbMonocular)
        {
            if(baseline<pKF2->mb)
            continue;
        }
        else
        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Compute Fundamental Matrix
        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false);

        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = kp1_ur>=0;

            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = kp2_ur>=0;

            // Check parallax between rays
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if(bStereo1)
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            cv::Mat x3D;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
            {
                // Linear Triangulation Method
                cv::Mat A(4,4,CV_32F);
                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                x3D = vt.row(3).t();

                if(x3D.at<float>(3)==0)
                    continue;

                // Euclidean coordinates
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);                
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
            else
                continue; //No stereo and very low parallax

            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;
            }
            else
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>5.991*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                continue;*/
            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            MapPoint* pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);

            pMP->AddObservation(mpCurrentKeyFrame,idx1);            
            pMP->AddObservation(pKF2,idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpMap->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);

            nnew++;
        }
    }
}

void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    int nn = 10;
    if(mbMonocular)
        nn=20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<KeyFrame*> vpTargetKFs;
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

        // Extend to some second neighbors
        const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }


    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        matcher.Fuse(pKFi,vpMapPointMatches);
    }

    // Search matches by projection from target KFs in current KF
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);


    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }
    LSDmatcher lineMatcher;
    vector<MapLine *> vpMapLineMatches = mpCurrentKeyFrame->GetMapLineMatches();
        //将当前帧的地图点分别投影到两级相邻关键帧，寻找匹配点对应的地图点进行融合，称为正向投影融合
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;
            lineMatcher.Fuse(pKFi, vpMapLineMatches);
        }
        //称为反向投影融合
    vector<MapLine *> vpLineFuseCandidates;
    vpLineFuseCandidates.reserve(vpTargetKFs.size() * vpMapLineMatches.size());

            //  Step 4.1：遍历每一个一级邻接和二级邻接关键帧，收集他们的地图点存储到 vpFuseCandidates
    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;
        vector<MapLine *> vpMapLinesKFi = pKFi->GetMapLineMatches();

             // 遍历当前一级邻接和二级邻接关键帧中所有的MapPoints,找出需要进行融合的并且加入到集合中
        for(vector<MapLine*>::iterator vitMP=vpMapLinesKFi.begin(), vendMP=vpMapLinesKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapLine* pML = *vitMP;
                if (!pML)
                    continue;

                if (pML->isBad() || pML->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                    continue;

                pML->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
                vpLineFuseCandidates.push_back(pML);
            }
        }

        lineMatcher.Fuse(mpCurrentKeyFrame, vpLineFuseCandidates);

        // Update Lines
        vpMapLineMatches = mpCurrentKeyFrame->GetMapLineMatches();
        for (auto pML : vpMapLineMatches)
         {
            if (pML) {
                if (!pML->isBad()) {
                    pML->ComputeDistinctiveDescriptors();
                    pML->UpdateAverageDir();
                }
            }
        }
    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}

cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        if(pKF->mnId==0)
            continue;
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }

                    nMPs++;
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = pKF->mvKeysUn[i].octave;
                        const map<KeyFrame*, size_t> observations = pMP->GetObservations();
                        int nObs=0;
                        for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        } 

        const vector<MapLine*> vpMapLines= pKF->GetMapLineMatches();

        // int nObs = 3;
        const int thObsline=2;
        int nRedundantObservationsLine=0;
        int nMLs=0;
        for(size_t i=0, iend=vpMapLines.size(); i<iend; i++)
        {
            MapLine* pML = vpMapLines[i];
            if(pML)
            {
                if(!pML->isBad())
                {
                    if(!mbMonocular)
                    {
                        if(pKF->mvDepthLineStart[i]>pKF->mThDepth || pKF->mvDepthLineStart[i]<0)
                            continue;
                        if(pKF->mvDepthLineEnd[i]>pKF->mThDepth || pKF->mvDepthLineEnd[i]<0)
                            continue;
                    }

                    nMLs++;
                    if(pML->Observations()>thObsline)
                    {
                        const int &scaleLevel = pKF->mvKeyLines[i].octave;
                        const map<KeyFrame*, size_t> observations = pML->GetObservations();
                        int nObs=0;
                        for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            const int &scaleLeveli = pKFi->mvKeyLines[mit->second].octave;

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>=thObsline)
                                    break;
                            }
                        }
                        if(nObs>=thObsline)
                        {
                            nRedundantObservationsLine++;
                        }
                    }
                }
            }
        }  

        if((nRedundantObservations+nRedundantObservationsLine)>0.9*(nMPs+nMLs))
            pKF->SetBadFlag();
    }
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
}

void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mlpRecentAddedMapLines.clear();
        mbResetRequested=false;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}
void LocalMapping::CreateNewMapLines() {
        // // Retrieve neighbor keyframes in covisibility graph
        int nn = 10;
        //step1：在当前关键帧的共视关键帧中找到共视成都最高的nn帧相邻帧vpNeighKFs
        const vector<KeyFrame *> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

        LSDmatcher lmatcher;
        // 获取当前帧的转换矩阵
        cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
        cv::Mat Rwc1 = Rcw1.t();
        cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
        cv::Mat Tcw1(3, 4, CV_32F);
        Rcw1.copyTo(Tcw1.colRange(0, 3));
        tcw1.copyTo(Tcw1.col(3));
        // 获取当前相机在世界坐标的位置
        cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();
        // 获取相机的内参
        const Mat &K1 = mpCurrentKeyFrame->mK;
        const float &fx1 = mpCurrentKeyFrame->fx;
        const float &fy1 = mpCurrentKeyFrame->fy;
        const float &cx1 = mpCurrentKeyFrame->cx;
        const float &cy1 = mpCurrentKeyFrame->cy;
        const float &invfx1 = mpCurrentKeyFrame->invfx;
        const float &invfy1 = mpCurrentKeyFrame->invfy;

        const float ratioFactor = 1.5f * mpCurrentKeyFrame->mfScaleFactor;
       
        // // 遍历相邻关键帧
        for (size_t i = 0; i < vpNeighKFs.size(); i++) {
            if (i > 0 && CheckNewKeyFrames())
                return;

            KeyFrame *pKF2 = vpNeighKFs[i];

            //邻居的关键帧在世界坐标系的位置
            cv::Mat Ow2 = pKF2->GetCameraCenter();
            // 获取邻居关键帧与当前帧的基线向量
            cv::Mat vBaseline = Ow2 - Ow1;
            // 求解范数
            const float baseline = cv::norm(vBaseline);
            // 判断相机的基线是否够长
            if (!mbMonocular) 
            {
                if (baseline < pKF2->mb)
                    continue;
            }
            else 
            {
                // / 邻接关键帧的场景深度中值
                const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
                 // baseline 与景深的比例
                const float ratioBaselineDepth = baseline / medianDepthKF2;
                 // 如果特别远（比例特别小），那么不考虑当前邻接的关键帧，不生成3D点
                if (ratioBaselineDepth < 0.01)
                    continue;
            }

        //     //计算基础矩阵
            cv::Mat F12 = ComputeF12(mpCurrentKeyFrame, pKF2);

        //     // Search matches that fulfill epipolar constraint
            vector<pair<size_t, size_t>> vMatchedIndices;
            lmatcher.SearchForTriangulation(mpCurrentKeyFrame, pKF2, vMatchedIndices);

            // 获取相邻关键帧的参数
            cv::Mat Rcw2 = pKF2->GetRotation();
            cv::Mat Rwc2 = Rcw2.t();
            cv::Mat tcw2 = pKF2->GetTranslation();
            cv::Mat Tcw2(3, 4, CV_32F);
            Rcw2.copyTo(Tcw2.colRange(0, 3));
            tcw2.copyTo(Tcw2.col(3));

            const Mat &K2 = pKF2->mK;
            const float &fx2 = pKF2->fx;
            const float &fy2 = pKF2->fy;
            const float &cx2 = pKF2->cx;
            const float &cy2 = pKF2->cy;
            const float &invfx2 = pKF2->invfx;
            const float &invfy2 = pKF2->invfy;

        //     // Triangulate each matched line Segment
        // 恢复每一个匹配成功的
            const int nmatches = vMatchedIndices.size();
            for (int ikl = 0; ikl < nmatches; ikl++) 
            {
                // 匹配对的当前帧特征线
                const int &idx1 = vMatchedIndices[ikl].first;
                // 匹配对的关键帧特征线
                const int &idx2 = vMatchedIndices[ikl].second;

                const KeyLine &kl1 = mpCurrentKeyFrame->mvKeyLines[idx1];
                bool bStereo1 = mpCurrentKeyFrame->mvDepthLineStart[idx1] > 0&&mpCurrentKeyFrame->mvuRightLineStart[idx1]>0;

                const KeyLine &kl2 = pKF2->mvKeyLines[idx2];
                bool bStereo2 = mpCurrentKeyFrame->mvDepthLineEnd[idx2]> 0&&mpCurrentKeyFrame->mvuRightLineEnd[idx2]>0;
                // // 获取直线的两个端点起点和终点在相机坐标系下
                // cv::Mat kl1sp, kl1ep, kl2sp, kl2ep;
                // kl1sp = (cv::Mat_<float>(3, 1) << (kl1.startPointX - cx1) * invfx1, (kl1.startPointY - cy1) *
                //                                                                     invfy1, 1.0);
                // kl1ep = (cv::Mat_<float>(3, 1) << (kl1.endPointX - cx1) * invfx1, (kl1.endPointY - cy1) * invfy1, 1.0);
                // kl2sp = (cv::Mat_<float>(3, 1) << (kl2.startPointX - cx2) * invfx2, (kl2.startPointY - cy2) *
                //                                                                     invfy2, 1.0);
                // kl2ep = (cv::Mat_<float>(3, 1) << (kl2.endPointX - cx2) * invfx2, (kl2.endPointY - cy2) * invfy2, 1.0);
                // 恢复世界坐标系
                cv::Mat sp3D, ep3D;
                if (bStereo1)
                 {
                    Vector6d line3D = mpCurrentKeyFrame->obtain3DLine(idx1);
                    sp3D = cv::Mat::eye(3, 1, CV_32F);
                    ep3D = cv::Mat::eye(3, 1, CV_32F);
                    sp3D.at<float>(0) = line3D(0);
                    sp3D.at<float>(1) = line3D(1);
                    sp3D.at<float>(2) = line3D(2);
                    ep3D.at<float>(0) = line3D(3);
                    ep3D.at<float>(1) = line3D(4);
                    ep3D.at<float>(2) = line3D(5);
                }
                 else if (bStereo2)
                  {
                    Vector6d line3D = pKF2->obtain3DLine(idx2);
                    sp3D = cv::Mat::eye(3, 1, CV_32F);
                    ep3D = cv::Mat::eye(3, 1, CV_32F);
                    sp3D.at<float>(0) = line3D(0);
                    sp3D.at<float>(1) = line3D(1);
                    sp3D.at<float>(2) = line3D(2);
                    ep3D.at<float>(0) = line3D(3);
                    ep3D.at<float>(1) = line3D(4);
                    ep3D.at<float>(2) = line3D(5);
                } else
                    continue; //No stereo and very low parallax

                cv::Mat sp3Dt = sp3D.t();
                cv::Mat ep3Dt = ep3D.t();


                //Check triangulation in front of cameras
                float zsp1 = Rcw1.row(2).dot(sp3Dt) + tcw1.at<float>(2);
                if (zsp1 <= 0)
                    continue;

                float zep1 = Rcw1.row(2).dot(ep3Dt) + tcw1.at<float>(2);
                if (zep1 <= 0)
                    continue;

                float zsp2 = Rcw2.row(2).dot(sp3Dt) + tcw2.at<float>(2);
                if (zsp2 <= 0)
                    continue;

                float zep2 = Rcw2.row(2).dot(ep3Dt) + tcw2.at<float>(2);
                if (zep2 <= 0)
                    continue;

                //检查冲投影误差
                const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kl1.octave];
                const float xsp1 = Rcw1.row(0).dot(sp3Dt) + tcw1.at<float>(0);
                const float ysp1 = Rcw1.row(1).dot(sp3Dt) + tcw1.at<float>(1);
                const float invzsp1 = 1.0 / zsp1;

                float usp1 = fx1 * xsp1 * invzsp1 + cx1;
                float vsp1 = fy1 * ysp1 * invzsp1 + cy1;

                float errXsp1 = usp1 - kl1.startPointX;
                float errYsp1 = vsp1 - kl1.startPointY;
                // 自己添加的右边图像误差
                Vector3d lC1 = mpCurrentKeyFrame->mvKeyLineFunctions[idx1];
                                 // 左边的误差  起点
                // float e1 = lC1(0) * xsp1 + lC1(1) * ysp1 + lC1(2);
                                // 反投影在像素
                float us1 = fx1*xsp1*invzsp1+cx1;
                float vs1 = fy1*ysp1*invzsp1+cy1;
                float est1sr=-(lC1(1)*vs1+lC1(2))/lC1(0);
                // / 获取当前线的起点
                 const float keyline1_urs=mpCurrentKeyFrame->mvuRightLineStart[idx1];
                float Err1sr=est1sr-keyline1_urs;


                #if false

                const float &sigmaSquare2 = mpCurrentKeyFrame->mvLevelSigma2[keyline1.octave];
                Vector3d lC1 = mpCurrentKeyFrame->mvKeyLineFunctions[idx1];
                const float x1 = Rcw1.row(0).dot(s3Dt) + tcw1.at<float>(0);
                const float y1 = Rcw1.row(1).dot(s3Dt) + tcw1.at<float>(1);
                const float invz1 = 1.0/SZC1;
                const float invz2 = 1.0/SZC2;
                const float invz3 = 1.0/EZC1;
                const float invz4 = 1.0/EZC2;
                // 左边的误差  起点
                float e1 = lC1(0) * x1 + lC1(1) * y1 + lC1(2);
                // 反投影在像素
                float us1 = fx1*x1*invz1+cx1;
                float vs1 = fy1*y1*invz1+cy1;
                float est1sr=-(lC1(1)*vs1+lC1(2))/lC1(0);
                float Err1sr=est1sr-keyline1_urs;
         

                const float x2 = Rcw1.row(0).dot(e3Dt) + tcw1.at<float>(0);
                const float y2 = Rcw1.row(1).dot(e3Dt) + tcw1.at<float>(1);
                // 直线函数终点
                float e2 = lC1(0) * x2 + lC1(1) * y2 + lC1(2);

                float ue1 = fx1*x2*invz1+cx1;
                float ve1 = fy1*y2*invz1+cy1;
                float est1er=-(lC1(1)*ve1+lC1(2))/lC1(0);
                float Err1er=est1er-keyline1_ure;

                #endif
                if ((errXsp1 * errXsp1 + errYsp1 * errYsp1+Err1sr*Err1sr) > 5.991* sigmaSquare1)
                    continue;

                const float xep1 = Rcw1.row(0).dot(ep3Dt) + tcw1.at<float>(0);
                const float yep1 = Rcw1.row(1).dot(ep3Dt) + tcw1.at<float>(1);
                const float invzep1 = 1.0 / zep1;

                float uep1 = fx1 * xep1 * invzep1 + cx1;
                float vep1 = fy1 * yep1 * invzep1 + cy1;

                float errXep1 = uep1 - kl1.endPointX;
                float errYep1 = vep1 - kl1.endPointY;

                // 自己添加的
                // float e2 = lC1(0) * xep1 + lC1(1) * yep1 + lC1(2);
                                // 反投影在像素
                float ue1 = fx1*xsp1*invzep1+cx1;
                float ve1 = fy1*ysp1*invzep1+cy1;
                float est1er=-(lC1(1)*ve1+lC1(2))/lC1(0);
                // / 获取当前线的起点
                 const float keyline1_ure=mpCurrentKeyFrame->mvuRightLineEnd[idx1];
                float Err1er=est1er-keyline1_ure;
                if ((errXep1 * errXep1 + errYep1 * errYep1+Err1er*Err1er) > 5.991 * sigmaSquare1)
                    continue;

                //Check reprojection error in second keyframe
                const float sigmaSquare2 = pKF2->mvLevelSigma2[kl2.octave];
                const float xsp2 = Rcw2.row(0).dot(sp3Dt) + tcw2.at<float>(0);
                const float ysp2 = Rcw2.row(1).dot(sp3Dt) + tcw2.at<float>(1);
                const float invzsp2 = 1.0 / zsp2;

                float usp2 = fx2 * xsp2 * invzsp2 + cx2;
                float vsp2 = fy2 * ysp2 * invzsp2 + cy2;
                float errXsp2 = usp2 - kl2.startPointX;
                float errYsp2 = vsp2 - kl2.startPointY;
                                // 自己添加的右边图像误差
                Vector3d lC2 = mpCurrentKeyFrame->mvKeyLineFunctions[idx2];
                                 // 左边的误差  起点
                // float e3 = lC2(0) * xsp2 + lC2(1) * lC2 + lC2(2);
                                // 反投影在像素
                float us2 = fx2*xsp2*invzsp2+cx2;
                float vs2 = fy2*ysp2*invzsp2+cy2;
                float est2sr=-(lC2(1)*vs2+lC2(2))/lC2(0);
                // / 获取当前线的起点
                 const float keyline2_urs=mpCurrentKeyFrame->mvuRightLineStart[idx2];
                float Err2sr=est2sr-keyline2_urs;
                if ((errXsp2 * errXsp2 + errYsp2 * errYsp2+Err2sr*Err2sr) > 5.991* sigmaSquare2)
                    continue;

                const float xep2 = Rcw2.row(0).dot(ep3Dt) + tcw2.at<float>(0);
                const float yep2 = Rcw2.row(1).dot(ep3Dt) + tcw2.at<float>(1);
                const float invzep2 = 1.0 / zep2;

                float uep2 = fx2 * xep2 * invzep2 + cx2;
                float vep2 = fy2 * yep2 * invzep2 + cy2;
                float errXep2 = uep2 - kl2.endPointX;
                float errYep2 = vep2 - kl2.endPointY;
                                             // 左边的误差  起点
                // float e4 = lC2(0) * xsp2 + lC2(1) * lC2 + lC2(2);
                                // 反投影在像素
                float ue2 = fx2*xep2*invzep2+cx2;
                float ve2 = fy2*yep2*invzep2+cy2;
                float est2er=-(lC2(1)*ve2+lC2(2))/lC2(0);
                // / 获取当前线的起点
                 const float keyline2_ure=mpCurrentKeyFrame->mvuRightLineEnd[idx2];
                float Err2er=est2er-keyline2_ure;
                if ((errXep2 * errXep2 + errYep2 * errYep2+Err2er*Err2er) > 5.991 * sigmaSquare2)
                    continue;

                cout<<errXep2 * errXep2 <<endl;
                //Check scale consistency
                cv::Mat normalsp1 = sp3D - Ow1;
                float distsp1 = cv::norm(normalsp1);

                cv::Mat normalep1 = ep3D - Ow1;
                float distep1 = cv::norm(normalep1);

                cv::Mat normalsp2 = sp3D - Ow2;
                float distsp2 = cv::norm(normalsp2);

                cv::Mat normalep2 = ep3D - Ow2;
                float distep2 = cv::norm(normalep2);

                if (distsp1 == 0 || distep1 == 0 || distsp2 == 0 || distep2 == 0)
                    continue;

                const float ratioDistsp = distsp2 / distsp1;
                const float ratioDistep = distep2 / distep1;
                const float ratioOctave =
                        mpCurrentKeyFrame->mvScaleFactors[kl1.octave] / pKF2->mvScaleFactors[kl2.octave];

                if (ratioDistsp * ratioFactor < ratioOctave || ratioDistsp > ratioOctave * ratioFactor ||
                    ratioDistep * ratioFactor < ratioOctave || ratioDistep > ratioOctave * ratioFactor)
                    continue;

                Vector6d line3D;
                line3D << sp3Dt.at<float>(0), sp3Dt.at<float>(1), sp3Dt.at<float>(2), ep3Dt.at<float>(0),
                        ep3Dt.at<float>(1), ep3Dt.at<float>(2);

                MapLine *pML = new MapLine(line3D, mpCurrentKeyFrame, mpMap);

                pML->AddObservation(mpCurrentKeyFrame, idx1);
                pML->AddObservation(pKF2, idx2);

                mpCurrentKeyFrame->AddMapLine(pML, idx1);
                pKF2->AddMapLine(pML, idx2);

                pML->ComputeDistinctiveDescriptors();
                pML->UpdateAverageDir();
                mpMap->AddMapLine(pML);

                mlpRecentAddedMapLines.push_back(pML);
            }

        }
    }
} //namespace ORB_SLAM
