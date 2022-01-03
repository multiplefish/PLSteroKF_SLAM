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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<pthread.h>
#include<mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    vector<KeyLine> vCurrentKeyLines;   //自己添加的，当前帧的特征线
    vector<KeyLine> vIniKeyLines;
    vector<bool> vbLineVO, vbLineMap;
    vector<float> DepthLineS,DepthLineE;
    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;

            // 线特征
            vCurrentKeyLines = mvCurrentKeyLines;
            vIniKeyLines = mvIniKeyLines;
            DepthLineS=mvDepthLineS;
            DepthLineE=mvDepthLineE;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
             // 线特征
            vCurrentKeyLines = mvCurrentKeyLines;
            vbLineVO = mvbLineVO;
            vbLineMap = mvbLineMap;
            DepthLineS=mvDepthLineS;
            DepthLineE=mvDepthLineE;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
            vCurrentKeyLines = mvCurrentKeyLines;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        mnTrackedLinesVO=0;
        mnLineTracked=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
        const int nl = vCurrentKeyLines.size();
        for(int i=0;i<nl;i++)
        {
            float zs=DepthLineS[i];
            float ze=DepthLineE[i];
            if(zs>0&&ze>0)
            {
            // cv::Point p1(vCurrentKeyLines[i].startPointX, vCurrentKeyLines[i].startPointY), p2(vCurrentKeyLines[i].endPointX, vCurrentKeyLines[i].endPointY);
            // // cv::circle(im,p1,2,cv::Scalar(0,0,255),8);
            // // cv::circle(im,p2,2,cv::Scalar(0,0,255),8);
            cv::line(im,vCurrentKeyLines[i].getStartPoint(),vCurrentKeyLines[i].getEndPoint(),cv::Scalar(0,0,255),2);   
            mnLineTracked++;
            }
   

        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
         int nMLs = mpMap->MapLinesInMap();
        s << "KFs: " << nKFs << "(MP ,ML): " <<"(" <<nMPs<<","<<nMLs <<")"<< 
        "Cur P  L: " << "("<<mnTracked<<","<<mnLineTracked<<")";

        // //在视觉里程计中匹配到的
        // if(mnTrackedPointsVO>0)
        //     s << ", (VO Pm, VO Lines matches): " << ",("<<mnTrackedPointsVO << mnTrackedLinesVO<<")";
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    //计算字符串文字所占用的图像区域的大小
    cv::Size textSize = cv::getTextSize(
        s.str(),                    //字符串
        cv::FONT_HERSHEY_PLAIN,     //字体
        1,                          //字体缩放
        1,                          //粗细
        &baseline);                 //基线,相对于最低端的文本点的,y坐标  //? 不是太明白
    //扩展图像
    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    //扩充区域填充黑色背景
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    //并且绘制文字
    cv::putText(
        imText,                         //目标图像
        s.str(),                        //要输出的文字
        cv::Point(5,imText.rows-5),     //输出文字的起始位置
        cv::FONT_HERSHEY_PLAIN,         //字体
        1,                              //缩放
        cv::Scalar(255,255,255),        //颜色,白色
        1,                              //线宽
        8);                             //线型

}

void FrameDrawer::Update(Tracking *pTracker)
{
  
    unique_lock<mutex> lock(mMutex);
    //拷贝跟踪线程的图像
    pTracker->mImGray.copyTo(mIm);
    //拷贝跟踪线程的特征点
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;

    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);

    mbOnlyTracking = pTracker->mbOnlyTracking;
    //线 特征
    mvCurrentKeyLines=pTracker->mCurrentFrame.mvKeylinesUn;

    mvDepthLineS=pTracker->mCurrentFrame.mvDepthLineStart;
    mvDepthLineE=pTracker->mCurrentFrame.mvDepthLineEnd;
    //获取线匹配关系
    // mvCurrentMatchim=pTracker->mCurrentFrame.outImg;
    // mvCurrentKeyLines = pTracker->mCurrentFrame.mvKeylinesUn;   //自己添加的

    NL = mvCurrentKeyLines.size();  //自己添加的
    mvbLineVO = vector<bool>(NL, false);
    mvbLineMap = vector<bool>(NL, false);
    //如果上一帧的时候,追踪器没有进行初始化
    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        //那么就要获取初始化帧的特征点和匹配信息
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;

        mvIniKeyLines=pTracker->mInitialFrame.mvKeylinesUn;

        // mvIniLineMatches=pTracker->mvIniLineMatches;
    }
  
    //如果上一帧是在正常跟踪
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        //获取当前帧地图点的信息
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    //该mappoints可以被多帧观测到，则为有效的地图点
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                    //否则表示这个特征点是在当前帧中第一次提取得到的点
                        mvbVO[i]=true;
                }
            }
        }

          for(int i=0;i<NL;i++)
        {
            MapLine* pMP = pTracker->mCurrentFrame.mvpMapLines[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbLineOutlier[i])
                {
                    //该mappoints可以被多帧观测到，则为有效的地图点
                    if(pMP->Observations()>0)
                        mvbLineMap[i]=true;
                    else
                    //否则表示这个特征点是在当前帧中第一次提取得到的点
                        mvbLineVO[i]=true;
                }
            }
        }
    }
    //更新追踪线程的跟踪状态
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM
