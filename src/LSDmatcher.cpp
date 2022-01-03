/**
* This file is part of Structure-SLAM.
* Copyright (C) 2020 Yanyan Li <yanyan.li at tum.de> (Technical University of Munich)
*
*/
#include "LSDmatcher.h"
#include "Converter.h"
using namespace std;
using namespace cv;
using namespace cv::line_descriptor;
using namespace Eigen;

#include<pthread.h>
namespace ORB_SLAM2
{
    const int LSDmatcher::TH_HIGH = 100;
    const int LSDmatcher::TH_LOW = 50;

    LSDmatcher::LSDmatcher(float nnratio, bool checkOri):mfNNratio(nnratio), mbCheckOrientation(checkOri)
    {
    }

    int LSDmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono) {
        int nmatches = 0;

            // Step 1 建立旋转直方图，用于检测旋转一致性
        vector<int> rotHist[30];
        for(int i=0;i<30;i++)
            rotHist[i].reserve(500);
        const float factor = 30/360.0f;

        const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
        const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0, 3).col(3);

        const cv::Mat twc = -Rcw.t()*tcw;

        const cv::Mat Rlw = LastFrame.mTcw.rowRange(0, 3).colRange(0, 3);
        const cv::Mat tlw = LastFrame.mTcw.rowRange(0, 3).col(3);

        const cv::Mat tlc = Rlw*twc+tlw;

        const bool bForward = tlc.at<float>(2)>CurrentFrame.mb && !bMono;
        const bool bBackward = -tlc.at<float>(2)>CurrentFrame.mb && !bMono;

        for (int i = 0; i < LastFrame.NL; i++) 
        {
            MapLine *pML = LastFrame.mvpMapLines[i];

            if (!pML || pML->isBad() || LastFrame.mvbLineOutlier[i]) {
                continue;
            }

            Vector6d P = pML->GetWorldPos();

            cv::Mat SP = (Mat_<float>(3, 1) << P(0), P(1), P(2));
            cv::Mat EP = (Mat_<float>(3, 1) << P(3), P(4), P(5));

            const cv::Mat SPc = Rcw * SP + tcw;
            const auto &SPcX = SPc.at<float>(0);
            const auto &SPcY = SPc.at<float>(1);
            const auto &SPcZ = SPc.at<float>(2);

            const cv::Mat EPc = Rcw * EP + tcw;
            const auto &EPcX = EPc.at<float>(0);
            const auto &EPcY = EPc.at<float>(1);
            const auto &EPcZ = EPc.at<float>(2);

            if (SPcZ < 0.0f || EPcZ < 0.0f)
                continue;

            const float invz1 = 1.0f / SPcZ;
            const float u1 = CurrentFrame.fx * SPcX * invz1 + CurrentFrame.cx;
            const float v1 = CurrentFrame.fy * SPcY * invz1 + CurrentFrame.cy;

            if (u1 < CurrentFrame.mnMinX || u1 > CurrentFrame.mnMaxX)
                continue;
            if (v1 < CurrentFrame.mnMinY || v1 > CurrentFrame.mnMaxY)
                continue;

            const float invz2 = 1.0f / EPcZ;
            const float u2 = CurrentFrame.fx * EPcX * invz2 + CurrentFrame.cx;
            const float v2 = CurrentFrame.fy * EPcY * invz2 + CurrentFrame.cy;

            if (u2 < CurrentFrame.mnMinX || u2 > CurrentFrame.mnMaxX)
                continue;
            if (v2 < CurrentFrame.mnMinY || v2 > CurrentFrame.mnMaxY)
                continue;

            int nLastOctave = LastFrame.mvKeylinesUn[i].octave;

            float radius = th*CurrentFrame.mvScaleFactors[nLastOctave];

            vector<size_t> vIndices;

            if(bForward)
                vIndices = CurrentFrame.GetLinesInArea(u1, v1, u2, v2, radius, nLastOctave);
            else if(bBackward)
                vIndices = CurrentFrame.GetLinesInArea(u1, v1, u2, v2, radius, 0, nLastOctave);
            else
                vIndices = CurrentFrame.GetLinesInArea(u1, v1, u2, v2,radius, nLastOctave-1, nLastOctave+1);

            if(vIndices.empty())
                continue;

            const cv::Mat desc = pML->GetDescriptor();

            int bestDist=256;
            int bestLevel= -1;

            int bestDist2=256;
            int bestLevel2 = -1;
            int bestIdx =-1 ;

            for(unsigned long idx : vIndices)
            {
                if( CurrentFrame.mvpMapLines[idx])
                    if( CurrentFrame.mvpMapLines[idx]->Observations()>0)
                        continue;
                if(CurrentFrame.mvuRightLineStart[idx])
                    {
                        // 双目和rgbd的情况，需要保证右图的点也在搜索半径以内
                        const float ur1 = u1 - CurrentFrame.mbf*invz1;
                        const float er1 = fabs(ur1 - CurrentFrame.mvuRightLineStart[idx]);
                        if(er1>radius)
                            continue;
                    }
                if(CurrentFrame.mvuRightLineStart[idx])
                    {
                        // 双目和rgbd的情况，需要保证右图的点也在搜索半径以内
                        const float ur2 = u2 - CurrentFrame.mbf*invz2;
                        const float er2 = fabs(ur2 - CurrentFrame.mvuRightLineStart[idx]);
                        if(er2>radius)
                            continue;
                    }

                const cv::Mat &d =  CurrentFrame.mLdesc.row(idx);

                const int dist = DescriptorDistance(desc, d);

                if(dist<bestDist)
                {
                    bestDist2 = bestDist;
                    bestDist = dist;
                    bestLevel2 = bestLevel;
                    bestLevel =  CurrentFrame.mvKeylinesUn[idx].octave;
                    bestIdx = idx;
                }
                else if(dist < bestDist2)
                {
                    bestLevel2 =  CurrentFrame.mvKeylinesUn[idx].octave;

                    bestDist2 = dist;
                }
            }

            if(bestDist <= TH_HIGH)
            {
                
                CurrentFrame.mvpMapLines[bestIdx]=pML;
                nmatches++;
                 // Step 6 计算匹配点旋转角度差所在的直方图
                    if(mbCheckOrientation)
                    {
                        float rot = LastFrame.mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestDist2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==30)
                            bin=0;
                        assert(bin>=0 && bin<30);
                        rotHist[bin].push_back(bestDist2);
                    }
                }
            }
        //Apply rotation consistency
    //  Step 7 进行旋转一致检测，剔除不一致的匹配
            if(mbCheckOrientation)
            {
                int ind1=-1;
                int ind2=-1;
                int ind3=-1;

                ComputeThreeMaxima(rotHist,30,ind1,ind2,ind3);

                for(int i=0; i<30; i++)
                {
                    // 对于数量不是前3个的点对，剔除
                    if(i!=ind1 && i!=ind2 && i!=ind3)
                    {
                        for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                        {
                            CurrentFrame.mvpMapLines[rotHist[i][j]]=static_cast<MapLine*>(NULL);
                            nmatches--;
                        }
                    }
                }
            }
        

        return nmatches;
    }
void LSDmatcher::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}
    int LSDmatcher::SearchByProjection(KeyFrame* pKF,Frame &currentF, vector<MapLine*> &vpMapLineMatches)
    {
        const vector<MapLine*> vpMapLinesKF = pKF->GetMapLineMatches();

        vpMapLineMatches = vector<MapLine*>(currentF.NL,static_cast<MapLine*>(NULL));

        int nmatches = 0;
        BFMatcher* bfm = new BFMatcher(NORM_HAMMING, false);
        Mat ldesc1, ldesc2;
        vector<vector<DMatch>> lmatches;
        ldesc1 = pKF->mLineDescriptors;
        ldesc2 = currentF.mLdesc;
        bfm->knnMatch(ldesc1, ldesc2, lmatches, 2);


        double nn_dist_th, nn12_dist_th;
        const float minRatio=1.0f/1.5f;
        // currentF.lineDescriptorMAD(lmatches, nn_dist_th, nn12_dist_th);
        currentF.mpLineSegment->lineDescriptorMAD(lmatches, nn_dist_th, nn12_dist_th);
        nn12_dist_th = nn12_dist_th*0.5;
        sort(lmatches.begin(), lmatches.end(), sort_descriptor_by_queryIdx());
        for(int i=0; i<lmatches.size(); i++)
        {
            //lmatches里装的 是匹配的编号，
            int qdx = lmatches[i][0].queryIdx;
            int tdx = lmatches[i][0].trainIdx;
            double dist_12 = lmatches[i][0].distance/lmatches[i][1].distance;
            if (lmatches[i][0].distance < 0.628* lmatches[i][1].distance) //0.6时候只有20对
            {
                MapLine* mapLine = vpMapLinesKF[qdx];

                if(mapLine)
                {

                    vpMapLineMatches[tdx]=mapLine;
                    nmatches++;
                }

            }
        }
        return nmatches;
    }
/**
 * @brief 通过投影地图点到当前帧，对Local MapPoint进行跟踪
 * 步骤
 * Step 1 遍历有效的局部地图点
 * Step 2 设定搜索搜索窗口的大小。取决于视角, 若当前视角和平均视角夹角较小时, r取一个较小的值
 * Step 3 通过投影点以及搜索窗口和预测的尺度进行搜索, 找出搜索半径内的候选匹配点索引
 * Step 4 寻找候选匹配点中的最佳和次佳匹配点
 * Step 5 筛选最佳匹配点
 * @param[in] F                         当前帧
 * @param[in] vpMapPoints               局部地图点，来自局部关键帧
 * @param[in] th                        搜索范围
 * @return int                          成功匹配的数目
 */
    int LSDmatcher::SearchByProjection(Frame &F, const std::vector<MapLine *> &vpMapLines, const float th)
    {
      int nmatches = 0;
// 如果 th！=1 (RGBD 相机或者刚刚进行过重定位), 需要扩大范围搜索
        const bool bFactor = th!=1.0;
        // Step 1 遍历有效的局部地图点
        for(size_t iMP=0; iMP<vpMapLines.size(); iMP++)
            {   
                 MapLine* pML= vpMapLines[iMP];
                        // 判断该点是否要投影
                if(!pML->mbTrackInView)
                    continue;

                if(pML->isBad())
                    continue;
            // 通过距离预测的金字塔层数，该层数相对于当前的帧
                 const int &nPredictedLevel = pML->mnTrackScaleLevel;
            // Step 2 设定搜索搜索窗口的大小。取决于视角, 若当前视角和平均视角夹角较小时, r取一个较小的值
                float r = RadiusByViewingCos(pML->mTrackViewCos); 
                // 如果需要扩大范围搜索，则乘以阈值th
                if(bFactor)
                    r*=th;
        // Step 3 通过投影点以及搜索窗口和预测的尺度进行搜索, 找出搜索半径内的候选匹配点索引
            vector<size_t> vIndices =
                    F.GetLinesInArea(pML->mTrackProjX1, pML->mTrackProjY1, pML->mTrackProjX2, pML->mTrackProjY2,
                                     r*F.mvScaleFactors[nPredictedLevel], nPredictedLevel-1, nPredictedLevel);
                // 没找到候选的,就放弃对当前点的匹配
                if(vIndices.empty())
                    continue;
                const cv::Mat MLdescriptor = pML->GetDescriptor();
                  // 最优的次优的描述子距离和index
                int bestDist=256;
                int bestLevel= -1;
                int bestDist2=256;
                int bestLevel2 = -1;
                int bestIdx =-1 ;
                 // Get best and second matches with near keypoints
                // Step 4 寻找候选匹配点中的最佳和次佳匹配点
                for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
                {
                    const size_t idx = *vit;

                    // 如果Frame中的该兴趣点已经有对应的MapPoint了,则退出该次循环
                if(F.mvpMapLines[idx])
                    if(F.mvpMapLines[idx]->Observations()>0)
                        continue;
                //如果是双目数据
                if(F.mvuRightLineStart[idx]>0&&F.mvuRightLineEnd[idx]>0)
                {
                    //计算在X轴上的投影误差
                    const float ers = fabs(pML->mTrackProjXRS-F.mvuRightLineStart[idx]);
                    const float ere = fabs(pML->mTrackProjXRE-F.mvuRightLineEnd[idx]);
                    //超过阈值,说明这个点不行,丢掉.
                    //这里的阈值定义是以给定的搜索范围r为参考,然后考虑到越近的点(nPredictedLevel越大), 相机运动时对其产生的影响也就越大,
                    //因此需要扩大其搜索空间.
                    //当给定缩放倍率为1.2的时候, mvScaleFactors 中的数据是: 1 1.2 1.2^2 1.2^3 ... 
                    if(ers>r*F.mvScaleFactors[nPredictedLevel]||ere>r*F.mvScaleFactors[nPredictedLevel])
                        continue;
                }
                const cv::Mat &d = F.mLdesc.row(idx);

                if(d.empty() || MLdescriptor.empty())
                    continue;
                // 计算地图点和候选投影点的描述子距离
                const int dist = DescriptorDistance(MLdescriptor,d);
                // 根据描述子寻找描述子距离最小和次小的特征点
                if(dist<bestDist)
                    {
                        bestDist2 = bestDist;
                        bestDist = dist;
                        bestLevel2 = bestLevel;
                        bestLevel = F.mvKeylinesUn[idx].octave;
                        bestIdx = idx;
                    }
                    else if(dist < bestDist2)
                    {
                        bestLevel2 = F.mvKeylinesUn[idx].octave;
                        bestDist2 = dist;
                    }
                }
             if(bestDist <= TH_HIGH)
            {
                if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                    continue;

                F.mvpMapLines[bestIdx]=pML;
                nmatches++;
            }

            }
              return nmatches;

    
    }

    int LSDmatcher::SerachForInitialize(Frame &InitialFrame, Frame &CurrentFrame, vector<pair<int, int>> &LineMatches)
    {
        LineMatches.clear();
        int nmatches = 0;
        BFMatcher* bfm = new BFMatcher(NORM_HAMMING, false);
        Mat ldesc1, ldesc2;
        vector<vector<DMatch>> lmatches;
        ldesc1 = InitialFrame.mLdesc;
        ldesc2 = CurrentFrame.mLdesc;
        
        bfm->knnMatch(ldesc1, ldesc2, lmatches, 2);

        double nn_dist_th, nn12_dist_th;
        // CurrentFrame.lineDescriptorMAD(lmatches, nn_dist_th, nn12_dist_th);
        CurrentFrame.mpLineSegment->lineDescriptorMAD(lmatches, nn_dist_th, nn12_dist_th);
        nn12_dist_th = nn12_dist_th*0.5;
        sort(lmatches.begin(), lmatches.end(), sort_descriptor_by_queryIdx());
        for(int i=0; i<lmatches.size(); i++)
        {
            int qdx = lmatches[i][0].queryIdx;
            int tdx = lmatches[i][0].trainIdx;
            double dist_12 = lmatches[i][1].distance - lmatches[i][0].distance;
            if(dist_12>nn12_dist_th)
            {
                LineMatches.push_back(make_pair(qdx, tdx));
                nmatches++;
            }
        }
        return nmatches;
    }

    int LSDmatcher::SearchByDescriptor(KeyFrame* pKF, Frame &currentF, vector<MapLine*> &vpMapLineMatches)
    {
        // 取出所有地图点
        const vector<MapLine*> vpMapLinesKF = pKF->GetMapLineMatches();
    
        // 将地图点置0
        vpMapLineMatches = vector<MapLine*>(currentF.NL,static_cast<MapLine*>(NULL));

        int nmatches = 0;
        BFMatcher* bfm = new BFMatcher(NORM_HAMMING, false);
        Mat ldesc1, ldesc2;
        vector<vector<DMatch>> lmatches;
        ldesc1 = pKF->mLineDescriptors;
        ldesc2 = currentF.mLdesc;
        // 匹配关键帧和当前帧的
        bfm->knnMatch(ldesc1, ldesc2, lmatches, 2);

        double nn_dist_th, nn12_dist_th;
        const float minRatio=1.0f/1.5f;
        // currentF.lineDescriptorMAD(lmatches, nn_dist_th, nn12_dist_th);
        currentF.mpLineSegment->lineDescriptorMAD(lmatches, nn_dist_th, nn12_dist_th);
        nn12_dist_th = nn12_dist_th*0.5;
        sort(lmatches.begin(), lmatches.end(), sort_descriptor_by_queryIdx());

        for(int i=0; i<lmatches.size(); i++)
        {
            //lmatches里装的 是匹配的编号，
            // 关键帧的匹配关系
            int qdx = lmatches[i][0].queryIdx;
             //当前帧的匹配关系
            int tdx = lmatches[i][0].trainIdx;
            double dist_12 = lmatches[i][0].distance/lmatches[i][1].distance;
            if(dist_12<minRatio)
            {
                // 地图的序号即地图线
                MapLine* mapLine = vpMapLinesKF[qdx];

                if(mapLine)
                {
                    vpMapLineMatches[tdx]=mapLine;
                    nmatches++;
                }

            }
        }
        return nmatches;
    }

    int LSDmatcher::SearchByDescriptorCuflast(Frame& pKF, Frame &currentF, vector<MapLine*> &vpMapLineMatches)
    {
        // 取出所有地图点
        const vector<MapLine*> vpMapLinesKF = pKF.mvpMapLines;
    
        // 将地图点置0
        vpMapLineMatches = vector<MapLine*>(currentF.NL,static_cast<MapLine*>(NULL));

        int nmatches = 0;
        BFMatcher* bfm = new BFMatcher(NORM_HAMMING, false);
        Mat ldesc1, ldesc2;
        vector<vector<DMatch>> lmatches;
        ldesc1 = pKF.mLdesc;
        ldesc2 = currentF.mLdesc;
        // 匹配关键帧和当前帧的
        bfm->knnMatch(ldesc1, ldesc2, lmatches, 2);

        double nn_dist_th, nn12_dist_th;
        const float minRatio=1.0f/1.5f;
        // currentF.lineDescriptorMAD(lmatches, nn_dist_th, nn12_dist_th);
        currentF.mpLineSegment->lineDescriptorMAD(lmatches, nn_dist_th, nn12_dist_th);
        nn12_dist_th = nn12_dist_th*0.5;
        sort(lmatches.begin(), lmatches.end(), sort_descriptor_by_queryIdx());

        for(int i=0; i<lmatches.size(); i++)
        {
            //lmatches里装的 是匹配的编号，
            // 关键帧的匹配关系
            int qdx = lmatches[i][0].queryIdx;
             //当前帧的匹配关系
            int tdx = lmatches[i][0].trainIdx;
            double dist_12 = lmatches[i][0].distance/lmatches[i][1].distance;
            if(dist_12<minRatio)
            {
                // 地图的序号即地图线
                MapLine* mapLine = vpMapLinesKF[qdx];

                if(mapLine)
                {
                    vpMapLineMatches[tdx]=mapLine;
                    nmatches++;
                }

            }
        }
        return nmatches;
    }

    int LSDmatcher::SearchByDescriptor(KeyFrame* pKF, KeyFrame *pKF2, vector<MapLine*> &vpMapLineMatches)
    {
        const vector<MapLine*> vpMapLinesKF = pKF->GetMapLineMatches();
        const vector<MapLine*> vpMapLinesKF2 = pKF2->GetMapLineMatches();

        if(vpMapLinesKF.size()==0||vpMapLinesKF2.size()==0) return 0;

        vpMapLineMatches = vector<MapLine*>(vpMapLinesKF.size(),static_cast<MapLine*>(NULL));
        int nmatches = 0;
        BFMatcher* bfm = new BFMatcher(NORM_HAMMING, false);
        Mat ldesc1, ldesc2;
        vector<vector<DMatch>> lmatches;
        ldesc1 = pKF->mLineDescriptors;
        ldesc2 = pKF2->mLineDescriptors;
        bfm->knnMatch(ldesc1, ldesc2, lmatches, 2);

        double nn_dist_th, nn12_dist_th;
        pKF2->lineDescriptorMAD(lmatches, nn_dist_th, nn12_dist_th);
        nn12_dist_th = nn12_dist_th*0.5;
        sort(lmatches.begin(), lmatches.end(), sort_descriptor_by_queryIdx());
        for(int i=0; i<lmatches.size(); i++)
        {
            int qdx = lmatches[i][0].queryIdx;
            int tdx = lmatches[i][0].trainIdx;
            double dist_12 = lmatches[i][1].distance - lmatches[i][0].distance;
            if(dist_12>nn12_dist_th)
            {
                MapLine* mapLine = vpMapLinesKF2[tdx];
                if(mapLine) {
                    vpMapLineMatches[qdx] = mapLine;
                    nmatches++;
                }
            }
        }
        return nmatches;
    }

    int LSDmatcher::DescriptorDistance(const Mat &a, const Mat &b)
    {
        const int *pa = a.ptr<int32_t>();
        const int *pb = b.ptr<int32_t>();

        int dist=0;

        for(int i=0; i<8; i++, pa++, pb++)
        {
            unsigned  int v = *pa ^ *pb;
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }

        return dist;
    }

    int LSDmatcher::SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2,
                                           vector<pair<size_t, size_t>> &vMatchedPairs)
    {
        vMatchedPairs.clear();
        int nmatches = 0;
        BFMatcher* bfm = new BFMatcher(NORM_HAMMING, false);
        Mat ldesc1, ldesc2;
        vector<vector<DMatch>> lmatches;
        ldesc1 = pKF1->mLineDescriptors;
        ldesc2 = pKF2->mLineDescriptors;
        bfm->knnMatch(ldesc1, ldesc2, lmatches, 2);

        double nn_dist_th, nn12_dist_th;
        pKF1->lineDescriptorMAD(lmatches, nn_dist_th, nn12_dist_th);
        nn12_dist_th = nn12_dist_th*0.1;
        sort(lmatches.begin(), lmatches.end(), sort_descriptor_by_queryIdx());
        for(int i=0; i<lmatches.size(); i++)
        {
            int qdx = lmatches[i][0].queryIdx;
            int tdx = lmatches[i][0].trainIdx;

            if (pKF1->GetMapLine(qdx) || pKF2->GetMapLine(tdx)) {
                continue;
            }

            double dist_12 = lmatches[i][1].distance - lmatches[i][0].distance;
            if(dist_12>nn12_dist_th)
            {
                vMatchedPairs.push_back(make_pair(qdx, tdx));
                nmatches++;
            }
        }
        return nmatches;
    }

    int LSDmatcher::Fuse(KeyFrame *pKF, const vector<MapLine *> &vpMapLines, const float th)
    {
      
        cv::Mat Rcw = pKF->GetRotation();
        cv::Mat tcw = pKF->GetTranslation();

        const float &fx = pKF->fx;
        const float &fy = pKF->fy;
        const float &cx = pKF->cx;
        const float &cy = pKF->cy;
        const float &bf = pKF->mbf;

        cv::Mat Ow = pKF->GetCameraCenter();
    
        int nFused=0;

        const int nLines = vpMapLines.size();

        // For each candidate MapPoint project and match
        for(int iML=0; iML<nLines; iML++)
        {
            MapLine* pML = vpMapLines[iML];

            // Discard Bad MapLines and already found
            if(!pML || pML->isBad())
                continue;

            Vector6d P = pML->GetWorldPos();

            cv::Mat SP = (Mat_<float>(3, 1) << P(0), P(1), P(2));
            cv::Mat EP = (Mat_<float>(3, 1) << P(3), P(4), P(5));

            const cv::Mat SPc = Rcw * SP + tcw;
            const auto &SPcX = SPc.at<float>(0);
            const auto &SPcY = SPc.at<float>(1);
            const auto &SPcZ = SPc.at<float>(2);

            const cv::Mat EPc = Rcw * EP + tcw;
            const auto &EPcX = EPc.at<float>(0);
            const auto &EPcY = EPc.at<float>(1);
            const auto &EPcZ = EPc.at<float>(2);

            if (SPcZ < 0.0f || EPcZ < 0.0f)
                continue;

            const float invz1 = 1.0f / SPcZ;
            const float u1 = fx * SPcX * invz1 + cx;
            const float v1 = fy * SPcY * invz1 + cy;

            if (u1 < pKF->mnMinX || u1 > pKF->mnMaxX)
                continue;
            if (v1 < pKF->mnMinY || v1 > pKF->mnMaxY)
                continue;

            const float invz2 = 1.0f / EPcZ;
            const float u2 = fx * EPcX * invz2 + cx;
            const float v2 = fy * EPcY * invz2 + cy;

            if (u2 < pKF->mnMinX || u2 > pKF->mnMaxX)
                continue;
            if (v2 < pKF->mnMinY || v2 > pKF->mnMaxY)
                continue;

            const float maxDistance = pML->GetMaxDistanceInvariance();
            const float minDistance = pML->GetMinDistanceInvariance();

            const cv::Mat OM = 0.5 * (SP + EP) - Ow;
            const float dist = cv::norm(OM);

            if (dist < minDistance || dist > maxDistance)
                continue;

            Vector3d Pn = pML->GetNormal();
            cv::Mat pn = (Mat_<float>(3, 1) << Pn(0), Pn(1), Pn(2));

            if(OM.dot(pn)<0.5*dist)
                continue;

            const int nPredictedLevel = pML->PredictScale(dist, pKF->mfLogScaleFactor);

            const float radius = th*pKF->mvScaleFactors[0];

            const vector<size_t> vIndices = pKF->GetLinesInArea(u1,v1, u2, v2, radius);

            if(vIndices.empty())
                continue;

            const cv::Mat dML = pML->GetDescriptor();

            int bestDist=INT_MAX;
            int bestIdx =-1 ;

            for(unsigned long idx : vIndices)
            {
                const int &klLevel = pKF->mvKeyLines[idx].octave;

                if(klLevel<nPredictedLevel-1 || klLevel>nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF->mLineDescriptors.row(idx);

                const int dist = DescriptorDistance(dML,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            if(bestDist<=TH_LOW)
            {
                MapLine* pMLinKF = pKF->GetMapLine(bestIdx);
                if(pMLinKF)
                {
                    if(!pMLinKF->isBad()) {
                        if(pMLinKF->Observations()>pML->Observations())
                            pML->Replace(pMLinKF);
                        else
                            pMLinKF->Replace(pML);
                    }
                }
                else
                {

                    pML->AddObservation(pKF,bestIdx);
                    pKF->AddMapLine(pML,bestIdx);
                }
                nFused++;
            }
        }

        return nFused;
    }

    float LSDmatcher::RadiusByViewingCos(const float &viewCos)
    {
        if(viewCos>0.998)
            return 5.0;
        else
            return 8.0;
    }

    int LSDmatcher::SearchByProjection(KeyFrame *pKF, cv::Mat Scw, const std::vector<MapLine *> &vpLines,
                                       std::vector<MapLine *> &vpMatched, int th) {
        // Get Calibration Parameters for later projection
        const float &fx = pKF->fx;
        const float &fy = pKF->fy;
        const float &cx = pKF->cx;
        const float &cy = pKF->cy;

        // Decompose Scw
        cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);
        const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
        cv::Mat Rcw = sRcw/scw;
        cv::Mat tcw = Scw.rowRange(0,3).col(3)/scw;
        cv::Mat Ow = -Rcw.t()*tcw;

         // 使用set类型，记录前面已经成功的匹配关系，避免重复匹配。并去除其中无效匹配关系（NULL）
        set<MapLine*> spAlreadyFound(vpMatched.begin(), vpMatched.end());
        spAlreadyFound.erase(static_cast<MapLine*>(NULL));

        int nmatches=0;

        //共视KF的所有地图点（不考虑当前KF已经匹配的地图点）投影到当前KF
        for(int iML=0, iendML=vpLines.size(); iML<iendML; iML++)
        {
            MapLine* pML = vpLines[iML];

            // Step 2.1 丢弃坏点，跳过当前KF已经匹配上的地图点
            if(!pML || pML->isBad() || spAlreadyFound.count(pML))
                continue;

             // Step 2.2 投影到当前KF的图像坐标并判断是否有效
            Vector6d P = pML->GetWorldPos();

            cv::Mat SP = (Mat_<float>(3, 1) << P(0), P(1), P(2));
            cv::Mat EP = (Mat_<float>(3, 1) << P(3), P(4), P(5));
            // 转换在相机坐标系下
            const cv::Mat SPc = Rcw * SP + tcw;
            const auto &SPcX = SPc.at<float>(0);
            const auto &SPcY = SPc.at<float>(1);
            const auto &SPcZ = SPc.at<float>(2);

            const cv::Mat EPc = Rcw * EP + tcw;
            const auto &EPcX = EPc.at<float>(0);
            const auto &EPcY = EPc.at<float>(1);
            const auto &EPcZ = EPc.at<float>(2);

            if (SPcZ < 0.0f || EPcZ < 0.0f)
                continue;
            // 转换在像素坐标系下
            const float invz1 = 1.0f / SPcZ;
            const float u1 = fx * SPcX * invz1 + cx;
            const float v1 = fy * SPcY * invz1 + cy;

            if (u1 < pKF->mnMinX || u1 > pKF->mnMaxX)
                continue;
            if (v1 < pKF->mnMinY || v1 > pKF->mnMaxY)
                continue;

            const float invz2 = 1.0f / EPcZ;
            const float u2 = fx * EPcX * invz2 + cx;
            const float v2 = fy * EPcY * invz2 + cy;

            if (u2 < pKF->mnMinX || u2 > pKF->mnMaxX)
                continue;
            if (v2 < pKF->mnMinY || v2 > pKF->mnMaxY)
                continue;
            // 判断距离是否在有效距离内
            const float maxDistance = pML->GetMaxDistanceInvariance();
            const float minDistance = pML->GetMinDistanceInvariance();
            // 地图点的中点到相机光心的向量
            const cv::Mat OM = 0.5 * (SP + EP) - Ow;
            const float dist = cv::norm(OM);

            if (dist < minDistance || dist > maxDistance)
                continue;
            // / /观察角度小于60°
            Vector3d Pn = pML->GetNormal();
            cv::Mat pn = (Mat_<float>(3, 1) << Pn(0), Pn(1), Pn(2));

            if(OM.dot(pn)<0.5*dist)
                continue;
// 根据当前这个地图点距离当前KF光心的距离,预测该点在当前KF中的尺度(图层)
            const int nPredictedLevel = pML->PredictScale(dist, pKF->mfLogScaleFactor);
// Search in a radius
        // 根据尺度确定搜索半径
            const float radius = th*pKF->mvScaleFactors[nPredictedLevel];
//  Step 2.3 搜索候选匹配点
            const vector<size_t> vIndices = pKF->GetLinesInArea(u1,v1, u2, v2, radius);

            if(vIndices.empty())
                continue;

            const cv::Mat dML = pML->GetDescriptor();

            int bestDist=256;
            int bestIdx =-1 ;
//  Step 2.4 遍历候选匹配点，找到最佳匹配点
            for(unsigned long idx : vIndices)
            {
                if(vpMatched[idx])
                    continue;

                const int &klLevel = pKF->mvKeyLines[idx].octave;

                if(klLevel<nPredictedLevel-1 || klLevel>nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF->mLineDescriptors.row(idx);

                const int dist = DescriptorDistance(dML,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }
// 该MapPoint与bestIdx对应的特征点匹配成功
            if(bestDist<=TH_LOW)
            {
                vpMatched[bestIdx]=pML;
                nmatches++;
            }
        }

        return nmatches;
    }

    int LSDmatcher::SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapLine *> &vpMatches12, const float &s12,
                                 const cv::Mat &R12, const cv::Mat &t12, const float th) {
        const float &fx = pKF1->fx;
        const float &fy = pKF1->fy;
        const float &cx = pKF1->cx;
        const float &cy = pKF1->cy;

        // Camera 1 from world
        cv::Mat R1w = pKF1->GetRotation();
        cv::Mat t1w = pKF1->GetTranslation();

        //Camera 2 from world
        cv::Mat R2w = pKF2->GetRotation();
        cv::Mat t2w = pKF2->GetTranslation();

        //Transformation between cameras
        cv::Mat sR12 = s12*R12;
        cv::Mat sR21 = (1.0/s12)*R12.t();
        cv::Mat t21 = -sR21*t12;

        const vector<MapLine*> vpMapLines1 = pKF1->GetMapLineMatches();
        const int N1 = vpMapLines1.size();

        const vector<MapLine*> vpMapLines2 = pKF2->GetMapLineMatches();
        const int N2 = vpMapLines2.size();

        vector<bool> vbAlreadyMatched1(N1,false);
        vector<bool> vbAlreadyMatched2(N2,false);

        for(int i=0; i<N1; i++)
        {
            MapLine* pML = vpMatches12[i];
            if(pML)
            {
                vbAlreadyMatched1[i]=true;
                int idx2 = pML->GetIndexInKeyFrame(pKF2);
                if(idx2>=0 && idx2<N2)
                    vbAlreadyMatched2[idx2]=true;
            }
        }

        vector<int> vnMatch1(N1,-1);
        vector<int> vnMatch2(N2,-1);

        // Transform from KF1 to KF2 and search
        for(int i1=0; i1<N1; i1++)
        {
            MapLine* pML = vpMapLines1[i1];

            if(!pML || pML->isBad() || vbAlreadyMatched1[i1])
                continue;

            Vector6d P = pML->GetWorldPos();

            cv::Mat SP = (Mat_<float>(3, 1) << P(0), P(1), P(2));
            cv::Mat EP = (Mat_<float>(3, 1) << P(3), P(4), P(5));

            const cv::Mat SPc1 = R1w * SP + t1w;
            const cv::Mat SPc2 = sR21 * SPc1 + t21;
            const auto &SPcX = SPc2.at<float>(0);
            const auto &SPcY = SPc2.at<float>(1);
            const auto &SPcZ = SPc2.at<float>(2);

            const cv::Mat EPc1 = R1w * EP + t1w;
            const cv::Mat EPc2 = sR21 * EPc1 + t21;
            const auto &EPcX = EPc2.at<float>(0);
            const auto &EPcY = EPc2.at<float>(1);
            const auto &EPcZ = EPc2.at<float>(2);

            if (SPcZ < 0.0f || EPcZ < 0.0f)
                continue;

            const float invz1 = 1.0f / SPcZ;
            const float u1 = fx * SPcX * invz1 + cx;
            const float v1 = fy * SPcY * invz1 + cy;

            if(!pKF2->IsInImage(u1,v1))
                continue;

            const float invz2 = 1.0f / EPcZ;
            const float u2 = fx * EPcX * invz2 + cx;
            const float v2 = fy * EPcY * invz2 + cy;

            if(!pKF2->IsInImage(u2,v2))
                continue;

            const float maxDistance = pML->GetMaxDistanceInvariance();
            const float minDistance = pML->GetMinDistanceInvariance();

            const float dist3D = cv::norm(0.5 * (SPc2 + EPc2));

            if (dist3D < minDistance || dist3D > maxDistance) {
                continue;
            }

            // Compute predicted octave
            const int nPredictedLevel = pML->PredictScale(dist3D, pKF2->mfLogScaleFactor);

            // Search in a radius
            const float radius = th*pKF2->mvScaleFactors[nPredictedLevel];

            const vector<size_t> vIndices = pKF2->GetLinesInArea(u1,v1,u2,v2,radius);

            if(vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius
            const cv::Mat dML = pML->GetDescriptor();

            int bestDist = INT_MAX;
            int bestIdx = -1;
            for(unsigned long idx : vIndices)
            {
                const int &klLevel = pKF2->mvKeyLines[idx].octave;

                if(klLevel<nPredictedLevel-1 || klLevel>nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF2->mLineDescriptors.row(idx);

                const int dist = DescriptorDistance(dML, dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            if(bestDist<=TH_HIGH)
            {
                vnMatch1[i1]=bestIdx;
            }
        }

        // Transform from KF2 to KF1 and search
        for(int i2=0; i2<N2; i2++)
        {
            MapLine* pML = vpMapLines2[i2];

            if(!pML || pML->isBad() || vbAlreadyMatched2[i2])
                continue;

            Vector6d P = pML->GetWorldPos();

            cv::Mat SP = (Mat_<float>(3, 1) << P(0), P(1), P(2));
            cv::Mat EP = (Mat_<float>(3, 1) << P(3), P(4), P(5));

            const cv::Mat SPc2 = R2w * SP + t2w;
            const cv::Mat SPc1 = sR12 * SPc2 + t12;
            const auto &SPcX = SPc1.at<float>(0);
            const auto &SPcY = SPc1.at<float>(1);
            const auto &SPcZ = SPc1.at<float>(2);

            const cv::Mat EPc2 = R2w * EP + t2w;
            const cv::Mat EPc1 = sR12 * EPc2 + t12;
            const auto &EPcX = EPc1.at<float>(0);
            const auto &EPcY = EPc1.at<float>(1);
            const auto &EPcZ = EPc1.at<float>(2);

            if (SPcZ < 0.0f || EPcZ < 0.0f)
                continue;

            const float invz1 = 1.0f / SPcZ;
            const float u1 = fx * SPcX * invz1 + cx;
            const float v1 = fy * SPcY * invz1 + cy;

            if(!pKF1->IsInImage(u1,v1))
                continue;

            const float invz2 = 1.0f / EPcZ;
            const float u2 = fx * EPcX * invz2 + cx;
            const float v2 = fy * EPcY * invz2 + cy;

            if(!pKF1->IsInImage(u2,v2))
                continue;

            const float maxDistance = pML->GetMaxDistanceInvariance();
            const float minDistance = pML->GetMinDistanceInvariance();

            const float dist3D = cv::norm(0.5 * (SPc1 + EPc1));

            if (dist3D < minDistance || dist3D > maxDistance)
                continue;

            // Compute predicted octave
            const int nPredictedLevel = pML->PredictScale(dist3D, pKF1->mfLogScaleFactor);

            // Search in a radius
            const float radius = th*pKF1->mvScaleFactors[nPredictedLevel];

            const vector<size_t> vIndices = pKF1->GetLinesInArea(u1,v1,u2,v2,radius);

            if(vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius
            const cv::Mat dML = pML->GetDescriptor();

            int bestDist = INT_MAX;
            int bestIdx = -1;
            for(unsigned long idx : vIndices)
            {
                const int &klLevel = pKF1->mvKeyLines[idx].octave;

                if(klLevel<nPredictedLevel-1 || klLevel>nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF1->mLineDescriptors.row(idx);

                const int dist = DescriptorDistance(dML, dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            if(bestDist<=TH_HIGH)
            {
                vnMatch2[i2]=bestIdx;
            }
        }

        // Check agreement
        int nFound = 0;

        for(int i1=0; i1<N1; i1++)
        {
            int idx2 = vnMatch1[i1];

            if(idx2>=0)
            {
                int idx1 = vnMatch2[idx2];
                if(idx1==i1)
                {
                    vpMatches12[i1] = vpMapLines2[idx2];
                    nFound++;
                }
            }
        }

        return nFound;
    }

    int LSDmatcher::Fuse(KeyFrame *pKF, cv::Mat Scw, const vector<MapLine *> &vpLines, float th,
                         vector<MapLine *> &vpReplaceLine) {
        // Get Calibration Parameters for later projection
        const float &fx = pKF->fx;
        const float &fy = pKF->fy;
        const float &cx = pKF->cx;
        const float &cy = pKF->cy;

        // Decompose Scw
        cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);
        const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
        cv::Mat Rcw = sRcw/scw;
        cv::Mat tcw = Scw.rowRange(0,3).col(3)/scw;
        cv::Mat Ow = -Rcw.t()*tcw;

        // Set of MapPoints already found in the KeyFrame
        const set<MapLine*> spAlreadyFound = pKF->GetMapLines();

        int nFused=0;

        const int nLines = vpLines.size();

        // For each candidate MapPoint project and match
        for(int iML=0; iML<nLines; iML++)
        {
            MapLine* pML = vpLines[iML];

            // Discard Bad MapPoints and already found
            if(!pML || pML->isBad() || spAlreadyFound.count(pML))
                continue;

            Vector6d P = pML->GetWorldPos();

            cv::Mat SP = (Mat_<float>(3, 1) << P(0), P(1), P(2));
            cv::Mat EP = (Mat_<float>(3, 1) << P(3), P(4), P(5));

            const cv::Mat SPc = Rcw * SP + tcw;
            const auto &SPcX = SPc.at<float>(0);
            const auto &SPcY = SPc.at<float>(1);
            const auto &SPcZ = SPc.at<float>(2);

            const cv::Mat EPc = Rcw * EP + tcw;
            const auto &EPcX = EPc.at<float>(0);
            const auto &EPcY = EPc.at<float>(1);
            const auto &EPcZ = EPc.at<float>(2);

            if (SPcZ < 0.0f || EPcZ < 0.0f)
                continue;

            const float invz1 = 1.0f / SPcZ;
            const float u1 = fx * SPcX * invz1 + cx;
            const float v1 = fy * SPcY * invz1 + cy;

            if (u1 < pKF->mnMinX || u1 > pKF->mnMaxX)
                continue;
            if (v1 < pKF->mnMinY || v1 > pKF->mnMaxY)
                continue;

            const float invz2 = 1.0f / EPcZ;
            const float u2 = fx * EPcX * invz2 + cx;
            const float v2 = fy * EPcY * invz2 + cy;

            if (u2 < pKF->mnMinX || u2 > pKF->mnMaxX)
                continue;
            if (v2 < pKF->mnMinY || v2 > pKF->mnMaxY)
                continue;

            const float maxDistance = pML->GetMaxDistanceInvariance();
            const float minDistance = pML->GetMinDistanceInvariance();

            const cv::Mat OM = 0.5 * (SP + EP) - Ow;
            const float dist = cv::norm(OM);

            if (dist < minDistance || dist > maxDistance)
                continue;

            Vector3d Pn = pML->GetNormal();
            cv::Mat pn = (Mat_<float>(3, 1) << Pn(0), Pn(1), Pn(2));

            if(OM.dot(pn)<0.5*dist)
                continue;

            const int nPredictedLevel = pML->PredictScale(dist, pKF->mfLogScaleFactor);

            const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

            const vector<size_t> vIndices = pKF->GetLinesInArea(u1,v1, u2, v2, radius);

            if(vIndices.empty())
                continue;

            const cv::Mat dML = pML->GetDescriptor();

            int bestDist=INT_MAX;
            int bestIdx =-1 ;

            for(unsigned long idx : vIndices)
            {
                const int &klLevel = pKF->mvKeyLines[idx].octave;

                if(klLevel<nPredictedLevel-1 || klLevel>nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF->mLineDescriptors.row(idx);

                const int dist = DescriptorDistance(dML,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            if(bestDist<=TH_LOW)
            {
                MapLine* pMLinKF = pKF->GetMapLine(bestIdx);
                if(pMLinKF)
                {
                    if(!pMLinKF->isBad())
                        vpReplaceLine[iML] = pMLinKF;
                }
                else
                {
                    pML->AddObservation(pKF,bestIdx);
                    pKF->AddMapLine(pML,bestIdx);
                }
                nFused++;
            }
        }

        return nFused;
    }

int LSDmatcher::SearchByProjection1(Frame &CurrentFrame, const Frame &LastFrame, const float th,const bool bMono)
{
    int nmatches = 0;

    // Rotation Histogram (to check rotation consistency)
    int HISTO_LENGTH=30;
    vector<int> rotHist[HISTO_LENGTH];  //HISTO_LENGTH=30
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = HISTO_LENGTH/360;

    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);

    const cv::Mat twc = -Rcw.t()*tcw;

    const cv::Mat Rlw = LastFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tlw = LastFrame.mTcw.rowRange(0,3).col(3);

    const cv::Mat tlc = Rlw*twc+tlw;

        // 判断前进还是后退
    const bool bForward = tlc.at<float>(2) > CurrentFrame.mb && !bMono;     // 非单目情况，如果Z大于基线，则表示相机明显前进
    const bool bBackward = -tlc.at<float>(2) > CurrentFrame.mb && !bMono;   // 非单目情况，如果-Z小于基线，则表示相机明显后退

    for(int i=0; i<LastFrame.NL; i++)
    {
        MapLine* pML = LastFrame.mvpMapLines[i];

        if(pML)
        {
            if(!LastFrame.mvbLineOutlier[i])
            {
                // Project
                Vector6d Lw = pML->GetWorldPos();
                Vector3d Lw1=Lw.head(3);
                Vector3d Lw2=Lw.tail(3);
                cv::Mat x3Dw1=Converter::toCvMat(Lw1);
                cv::Mat x3Dw2=Converter::toCvMat(Lw2);

                cv::Mat x3Dc1 = Rcw*x3Dw1+tcw;
                cv::Mat x3Dc2 = Rcw*x3Dw2+tcw;



                const float xc1 = x3Dc1.at<float>(0);
                const float yc1 = x3Dc1.at<float>(1);
                const float invzc1 = 1.0/x3Dc1.at<float>(2);

                const float xc2 = x3Dc2.at<float>(0);
                const float yc2 = x3Dc2.at<float>(1);
                const float invzc2 = 1.0/x3Dc2.at<float>(2);

                if(invzc1<0|| invzc2<0)
                    continue;
                 // 投影到当前帧中
                float u1 = CurrentFrame.fx*xc1*invzc1+CurrentFrame.cx;
                float v1 = CurrentFrame.fy*yc1*invzc1+CurrentFrame.cy;

                if(u1<CurrentFrame.mnMinX || u1>CurrentFrame.mnMaxX)
                    continue;
                if(v1<CurrentFrame.mnMinY || v1>CurrentFrame.mnMaxY)
                    continue;
                                 // 投影到当前帧中
                float u2 = CurrentFrame.fx*xc2*invzc2+CurrentFrame.cx;
                float v2 = CurrentFrame.fy*yc2*invzc2+CurrentFrame.cy;

                if(u2<CurrentFrame.mnMinX || u2>CurrentFrame.mnMaxX)
                    continue;
                if(v2<CurrentFrame.mnMinY || v2>CurrentFrame.mnMaxY)
                    continue;


                int nLastOctave = LastFrame.mvKeylinesUn[i].octave;

                // Search in a window. Size depends on scale
                float radius = th;

                vector<size_t> vIndices2;

                // 以下可以这么理解，例如一个有一定面积的圆点，在某个尺度n下它是一个特征点
                // 当相机前进时，圆点的面积增大，在某个尺度m下它是一个特征点，由于面积增大，则需要在更高的尺度下才能检测出来
                // 当相机后退时，圆点的面积减小，在某个尺度m下它是一个特征点，由于面积减小，则需要在更低的尺度下才能检测出来
                if(bForward) // 前进,则上一帧兴趣点在所在的尺度nLastOctave<=nCurOctave
                    vIndices2 = CurrentFrame.GetLinesInArea(u1,v1,u2,v2, radius, nLastOctave-1,nLastOctave+1);
                else if(bBackward) // 后退,则上一帧兴趣点在所在的尺度0<=nCurOctave<=nLastOctave
                    vIndices2 = CurrentFrame.GetLinesInArea(u1,v1,u2,v2, radius, 0, nLastOctave);
                else // 在[nLastOctave-1, nLastOctave+1]中搜索
                    vIndices2 = CurrentFrame.GetLinesInArea(u1,v1,u2,v2, radius, nLastOctave-1, nLastOctave+1);
                // cout<<"vIndices2"<<vIndices2.size()<<endl;
                if(vIndices2.empty())
                    continue;
   

                const cv::Mat dML = pML->GetDescriptor();

                int bestDist = 256;
                int bestLevel= -1;
                int bestDist2=256;
                int bestLevel2 = -1;
                int bestIdx2 = -1;

                for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                {
                    const size_t i2 = *vit;

                    if(CurrentFrame.mvpMapLines[i2])
                        if(CurrentFrame.mvpMapLines[i2]->Observations()>0)
                            continue;
                    
                    if(CurrentFrame.mvuRightLineStart[i2]>0&&CurrentFrame.mvuRightLineEnd[i2]>0)
                    {
                        // 双目和rgbd的情况，需要保证右图的点也在搜索半径以内
                        const float ur1 = u1 - CurrentFrame.mbf*invzc1;
                        const float er1 = fabs(ur1 - CurrentFrame.mvuRightLineStart[i2]);
                        if(er1>radius)
                            continue;
                        const float ur2 = u2 - CurrentFrame.mbf*invzc2;
                        const float er2 = fabs(ur2 - CurrentFrame.mvuRightLineStart[i2]);
                        if(er2>radius)
                            continue;
                    }
                    // cout<<" sss"<<i2<<endl;
                    const cv::Mat &d = CurrentFrame.mLdesc.row(i2);

                    const int dist = DescriptorDistance(dML,d);

                    // float max_ =  std::max(LastFrame.mvKeylinesUn[i].lineLength , CurrentFrame.mvKeylinesUn[i2].lineLength);
                    // float min_ =  std::min(LastFrame.mvKeylinesUn[i].lineLength , CurrentFrame.mvKeylinesUn[i2].lineLength);

                    // if(min_ / max_ < 0.5)
                    //     continue;

                    if(dist<bestDist)
                    {
                        bestDist2=bestDist;
                        bestDist=dist;
                        bestIdx2=i2;
                        bestLevel2 = bestLevel;
                        bestLevel = CurrentFrame.mvKeylinesUn[i].octave;

                    }
                    else if(dist<bestDist2)
                    {
                        bestLevel2 = CurrentFrame.mvKeylinesUn[i].octave;
                        bestDist2=dist;
                    }
                }
                // cout<<"bestDist"<<bestDist<<endl;
                if(bestDist<=130)
                {
                    if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                    continue;
                    CurrentFrame.mvpMapLines[bestIdx2]=pML;
                    nmatches++;
                    // cout<<"nmatches"<<nmatches<<endl;



//   if(dist<bestDist)
//             {
//                 bestDist2=bestDist;
//                 bestDist=dist;
//                 bestLevel2 = bestLevel;
//                 bestLevel = F.mvKeysUn[idx].octave;
//                 bestIdx=idx;
//             }
//             else if(dist<bestDist2)
//             {
//                 bestLevel2 = F.mvKeysUn[idx].octave;
//                 bestDist2=dist;
//             }
//         }

//         // Apply ratio to second match (only if best and second are in the same scale level)
//         if(bestDist<=TH_HIGH)
//         {
//             if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
//                 continue;

//             F.mvpMapPoints[bestIdx]=pMP;
//             nmatches++;
//         }
                    //  // Step 6 计算匹配点旋转角度差所在的直方
                    // float rot = LastFrame.mvKeylinesUn[i].angle-CurrentFrame.mvKeylinesUn[i].angle;
                    // if(rot<0.0)
                    //     rot+=360.0f;
                    // int bin = round(rot*factor);
                    // if(bin==HISTO_LENGTH)
                    //     bin=0;
                    // assert(bin>=0 && bin<HISTO_LENGTH);
                    // rotHist[bin].push_back(bestIdx2);
                    

                }
            }
        }
    }
     
        // int ind1=-1;
        // int ind2=-1;
        // int ind3=-1;

        // ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        // for(int i=0; i<HISTO_LENGTH; i++)
        // {
        //         // 对于数量不是前3个的点对，剔除
        //     if(i!=ind1 && i!=ind2 && i!=ind3)
        //     {
        //         for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
        //         {
        //                 CurrentFrame.mvpMapLines[rotHist[i][j]]=static_cast<MapLine*>(NULL);
        //                 nmatches--;
        //         }
        //     }
        // }
        

    return nmatches;
}


int LSDmatcher::SearchByProjection2(Frame &CurrentFrame, const Frame &LastFrame, const float th)
{
    int nmatches = 0;

    // Rotation Histogram (to check rotation consistency)
    int HISTO_LENGTH=30;
    vector<int> rotHist[HISTO_LENGTH];  //HISTO_LENGTH=30
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = HISTO_LENGTH/360.0f;

    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);

    const cv::Mat twc = -Rcw.t()*tcw;

    const cv::Mat Rlw = LastFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tlw = LastFrame.mTcw.rowRange(0,3).col(3);

    const cv::Mat tlc = Rlw*twc+tlw;

    for(int i=0; i<LastFrame.NL; i++)
    {
        MapLine* pML = LastFrame.mvpMapLines[i];

        if(pML)
        {
            if(!LastFrame.mvbLineOutlier[i])
            {
                // Project
                Vector6d Lw = pML->GetWorldPos();

                if(!CurrentFrame.isInFrustum(pML, 0.5))
                    continue;

                int nLastOctave = pML->mnTrackScaleLevel;

                // Search in a window. Size depends on scale
                float radius = th;

                vector<size_t> vIndices2;

                // vIndices2 = CurrentFrame.GetFeaturesInAreaForLine(pML->mTrackProjX1, pML->mTrackProjY1, pML->mTrackProjX2, pML->mTrackProjY2,
                //                      radius, nLastOctave-1, nLastOctave+1, 0.96);
                vIndices2 = CurrentFrame.GetLinesInArea(pML->mTrackProjX1, pML->mTrackProjY1, pML->mTrackProjX2, pML->mTrackProjY2, radius, nLastOctave-1, nLastOctave+1);
                
                if(vIndices2.empty())
                    continue;

                const cv::Mat dML = pML->GetDescriptor();

                int bestDist = 256;
                int bestIdx2 = -1;

                for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                {
                    const size_t i2 = *vit;
                    if(CurrentFrame.mvpMapLines[i2])
                        if(CurrentFrame.mvpMapLines[i2]->Observations()>0)
                            continue;

                    const cv::Mat &d = CurrentFrame.mLdesc.row(i2);

                    const int dist = DescriptorDistance(dML,d);

                    float max_ =  std::max(LastFrame.mvKeylinesUn[i].lineLength , CurrentFrame.mvKeylinesUn[i2].lineLength);
                    float min_ =  std::min(LastFrame.mvKeylinesUn[i].lineLength , CurrentFrame.mvKeylinesUn[i2].lineLength);

                    if(min_ / max_ < 0.75)
                        continue;

                    if(dist<bestDist)
                    {
                        bestDist=dist;
                        bestIdx2=i2;
                    }
                }

                if(bestDist<=TH_HIGH)
                {
                    CurrentFrame.mvpMapLines[bestIdx2]=pML;
                    nmatches++;

                    // /////////////////////////////////////////////////////////////////////////////////////////////////////
                    // cv::Point Point_1, Point_2;
                    // KeyLine line_1 = LastFrame.mvKeylinesUn[i];
                    // KeyLine line_2 = CurrentFrame.mvKeylinesUn[bestIdx2];
                    // Point_1.x = (line_1.startPointX + line_1.endPointX)/2.0;
                    // Point_1.y = (line_1.startPointY + line_1.endPointY)/2.0;
                    // Point_2.x = (line_2.startPointX + line_2.endPointX)/2.0 + CurrentFrame.ImageGray.cols;
                    // Point_2.y = (line_2.startPointY + line_2.endPointY)/2.0;
                    // cv::line(pic_Temp, Point_1, Point_2, Scalar(0,0,255), 1, CV_AA);
                    // /////////////////////////////////////////////////////////////////////////////////////////////////////

                }
            }
        }
    }

    // cv::imwrite("./matchResultTrack.jpg", pic_Temp);

    return nmatches;
}
}
