/*
 * @Descripttion: 
 * @version: 
 * @Author: Chikyu
 * @Date: 2021-11-11 23:22:39
 * @LastEditTime: 2021-12-22 23:56:34
 */
/**
 with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORB_SLAM2_LSDMATCHER_H
#define ORB_SLAM2_LSDMATCHER_H

#include "MapLine.h"
#include "KeyFrame.h"
#include "Frame.h"

namespace ORB_SLAM2
{
    class LSDmatcher
    {
    public:
        static const int TH_HIGH, TH_LOW;

        LSDmatcher(float nnratio=0.6, bool checkOri=true);

        int SearchByDescriptor(KeyFrame* pKF, Frame &currentF, std::vector<MapLine*> &vpMapLineMatches);
        int SearchByDescriptorCuflast(Frame& pKF, Frame &currentF, std::vector<MapLine*> &vpMapLineMatches);
        int SearchByDescriptor(KeyFrame* pKF, KeyFrame *pKF2, std::vector<MapLine*> &vpMapLineMatches);
        int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);
        int SearchByProjection(KeyFrame* pKF,Frame &currentF, vector<MapLine*> &vpMapLineMatches);
        int SearchByProjection(Frame &F, const std::vector<MapLine*> &vpMapLines, const float th=3);
        int SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapLine*> &vpLines, std::vector<MapLine*> &vpMatched, int th);
        int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapLine *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);
        int SerachForInitialize(Frame &InitialFrame, Frame &CurrentFrame, std::vector<std::pair<int,int>> &LineMatches);
        int SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<std::pair<size_t, size_t>> &vMatchedPairs);
        void ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);
        int SearchByProjection1(Frame &CurrentFrame, const Frame &LastFrame, const float th,const bool bMono);
        int SearchByProjection2(Frame &CurrentFrame, const Frame &LastFrame, const float th);

        // Project MapLines into KeyFrame and search for duplicated MapLines
        int Fuse(KeyFrame* pKF, const vector<MapLine *> &vpMapLines, const float th=3.0);

        int Fuse(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapLine*> &vpLines, float th, vector<MapLine *> &vpReplaceLine);

        static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

    protected:
        float RadiusByViewingCos(const float &viewCos);
        float mfNNratio;
        bool mbCheckOrientation;
    };
}


#endif //ORB_SLAM2_LSDMATCHER_H
