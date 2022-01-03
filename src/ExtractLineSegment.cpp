
#include "ExtractLineSegment.h"
#include <memory>
#include <opencv2/line_descriptor/descriptor.hpp>
#include <unordered_map>

namespace ORB_SLAM2
{
LineSegment::LineSegment()
{
    
}
LineSegment::LineSegment(int LSDlines)
{
    lines=LSDlines;

}

void LineSegment::ExtractLineSegment(const Mat &img, vector<cv::line_descriptor::KeyLine> &keylines,int _line)
{
  // cv::Rect rect(10, 10, img., height);
      // cv::Mat Temp ;
      // img(Range(10,(img.rows-10)),Range(10,(img.cols-10))).copyTo(Temp);
      Ptr<cv::line_descriptor::LSDDetector> lsd = cv::line_descriptor::LSDDetector::createLSDDetector();
      lsd->detect(img, keylines, 1.2, 1);


      unsigned int lsdNFeatures = _line;
      // filter lines
      // 非常多的线
      sort(keylines.begin(), keylines.end(),sort_lines_by_response());
      if(keylines.size()>lsdNFeatures)
          keylines.resize(lsdNFeatures);
    int index=keylines.size();
    for(unsigned int i=0; i<keylines.size(); i++)
    {
        if(keylines[i].lineLength>=10 && keylines[i+1].lineLength<10)
        {
            index = i;
            break;
        }
    }
    keylines.resize(index + 1);
    for(unsigned int i=0; i<index + 1; ++i)
    {
        keylines[i].class_id = i;
    }




}
// 直线提取
void LineSegment::ExtractLineSegment(const Mat &img, vector<cv::line_descriptor::KeyLine> &keylines, Mat &ldesc, vector<Eigen::Vector3d> &keylineFunctions,int _lines,
int scale, int numOctaves)
{
    cv::Mat Temp ;
    img.copyTo(Temp);
    shared_ptr<LineSegment>linelsd;


    linelsd->ExtractLineSegment(Temp, keylines,_lines);



    vector<Eigen::Matrix<double,2,1> >  PolarLine;
    PolarLine=linelsd->PolarLineSegment(keylines);
    linelsd->getIndexWithPolarLine(Temp,PolarLine,keylines);



    cv::Mat  imgclone;
    GaussianBlur(Temp,imgclone,Size(7, 7),2,2,BORDER_REFLECT_101);//边缘拓展点插值类型
    Ptr<cv::line_descriptor::BinaryDescriptor> lbd = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
    lbd->compute(imgclone, keylines, ldesc);     //计算特征线段的描述子

    // 计算特征线段所在直线的系数
    for(vector<KeyLine>::iterator it=keylines.begin(); it!=keylines.end(); ++it)
    {
        Vector3d sp_l; sp_l << it->startPointX, it->startPointY, 1.0;
        Vector3d ep_l; ep_l << it->endPointX, it->endPointY, 1.0;
        Vector3d lineF;     //直线方程
        lineF << sp_l.cross(ep_l);
        lineF = lineF / sqrt(lineF(0)*lineF(0)+lineF(1)*lineF(1));
        keylineFunctions.push_back(lineF);
    }
}

void LineSegment::LineSegmentMatch(Mat &ldesc_1, Mat &ldesc_2)
{
    BFMatcher bfm(NORM_HAMMING, false);
    bfm.knnMatch(ldesc_1, ldesc_2, line_matches, 2);
}

void LineSegment::lineDescriptorMAD( vector<vector<DMatch>> line_matches, double &nn_mad, double &nn12_mad) const
{
    vector<vector<DMatch>> matches_nn, matches_12;
    matches_nn = line_matches;
    matches_12 = line_matches;

    // estimate the NN's distance standard deviation
    double nn_dist_median;
    sort( matches_nn.begin(), matches_nn.end(), compare_descriptor_by_NN_dist());
    nn_dist_median = matches_nn[int(matches_nn.size()/2)][0].distance;
    for(unsigned int i=0; i<matches_nn.size(); i++)
        matches_nn[i][0].distance = fabsf(matches_nn[i][0].distance - nn_dist_median);
    sort(matches_nn.begin(), matches_nn.end(), compare_descriptor_by_NN_dist());
    nn_mad = 1.4826 * matches_nn[int(matches_nn.size()/2)][0].distance;

    // estimate the NN's 12 distance standard deviation
    double nn12_dist_median;
    sort( matches_12.begin(), matches_12.end(), conpare_descriptor_by_NN12_dist());
    nn12_dist_median = matches_12[int(matches_12.size()/2)][1].distance - matches_12[int(matches_12.size()/2)][0].distance;
    for (unsigned int j=0; j<matches_12.size(); j++)
        matches_12[j][0].distance = fabsf( matches_12[j][1].distance - matches_12[j][0].distance - nn12_dist_median);
    sort(matches_12.begin(), matches_12.end(), compare_descriptor_by_NN_dist());
    nn12_mad = 1.4826 * matches_12[int(matches_12.size()/2)][0].distance;
}

double LineSegment::lineSegmentOverlap(double spl_obs, double epl_obs, double spl_proj, double epl_proj)
{
    double sln    = min(spl_obs,  epl_obs);     //线特征两个端点的观察值的最小值
    double eln    = max(spl_obs,  epl_obs);     //线特征两个端点的观察值的最大值
    double spn    = min(spl_proj, epl_proj);    //线特征两个端点的投影点的最小值
    double epn    = max(spl_proj, epl_proj);    //线特征两个端点的投影点的最大值

    double length = eln-spn;

    double overlap;
    if ( (epn < sln) || (spn > eln) )
        overlap = 0.f;
    else{
        if ( (epn>eln) && (spn<sln) )
            overlap = eln-sln;
        else
            overlap = min(eln,epn) - max(sln,spn);
    }

    if(length>0.01f)
        overlap = overlap / length;
    else
        overlap = 0.f;

    return overlap;
}




vector<Eigen::Matrix<double,2,1> > LineSegment::PolarLineSegment(vector<cv::line_descriptor::KeyLine> &keylines)
{
   vector<Eigen::Matrix<double,2,1> >  Relust;
   Eigen::Matrix<double,2,1> Polarline;
   for(int i = 0; i < keylines.size(); i++)
   {

    if(fabs(keylines[i].startPointX-keylines[i].endPointX) < 1e-5 )//垂直直线
    {
        if(keylines[i].startPointX> 0){
            Polarline<<keylines[i].startPointX,0;
            Relust.push_back(Polarline);
        }
        else{
            Polarline<<keylines[i].startPointX,CV_PI;
            Relust.push_back(Polarline);
        }
    }

    if(fabs(keylines[i].startPointY-keylines[i].endPointY) < 1e-5 ) //水平直线
    {
        if(keylines[i].startPointY > 0){
            Polarline<<keylines[i].startPointY,CV_PI/2;
            Relust.push_back(Polarline);
        }
        else{
            Polarline<<keylines[i].startPointY,3*CV_PI/2;
            Relust.push_back(Polarline);
        }
    }
    // 计算写率
    float k = (keylines[i].startPointY-keylines[i].endPointY)/(keylines[i].startPointX-keylines[i].endPointX);
    float b =  keylines[i].startPointY - k*keylines[i].startPointX;
    float theta;
    // 2象限
    if( k < 0 && b > 0)
        theta =  atan(-1/k);
    // 1象限
    else if( k > 0 && b > 0)
        theta = CV_PI + atan(-1/k);
    // 4象限
    else if( k< 0 && b < 0)
        theta = CV_PI + atan(-1/k);
    // 3象限
    else if( k> 0 && b < 0)
        theta = 2*CV_PI +atan(-1/k);
    //及坐标表示

    float r = keylines[i].startPointX*cos(theta)+ keylines[i].startPointY*sin(theta);
    Polarline<<r,theta;
    Relust.push_back(Polarline); 
   }
   return Relust;
      
}

void LineSegment::getIndexWithPolarLine(const Mat &img,vector<Eigen::Matrix<double,2,1> >&polarLines,vector<cv::line_descriptor::KeyLine> &keylines)
{

    shared_ptr<LineSegment>linelsd;
    vector<int>_index;
    for(int i=0;i<polarLines.size();i++)
        _index.push_back(i);

    int polar_num = polarLines.size();
    for (int i=0; i < polar_num -1;i++)
    {
        if(keylines[i].response==-1) continue;
        float minTheta = CV_PI;
        float minR = 50;
        Eigen::Matrix<double,2,1>  polar1 = polarLines[i];
        for (int j = i+1; j < polar_num; j++)
        {
            Eigen::Matrix<double,2,1>  polar2 = polarLines[j];
            float dTheta = fabs(polar2(1) - polar1(1));
            float dR = fabs(polar2(0) - polar1(0));
            //获取最新的RO和THeta
            if(dTheta < minTheta )
                minTheta = dTheta;
            if(dR < minR)
                minR = dR;
            //同类直线角度误差不超过1.°，距离误差不超过1%
            if(dTheta < 1.0f*CV_PI/180.0f && dR < polar1[0]*0.01f)
            {
                if(keylines[j].response==-1) continue;
                // 合并两条直线
                std::vector<cv::Point> fit1;
                cv::Point MaxP,MinP;
                fit1.push_back(cv::Point(keylines[i].getStartPoint().x, keylines[i].getStartPoint().y));
                fit1.push_back(cv::Point(keylines[i].getEndPoint().x, keylines[i].getEndPoint().y));
                fit1.push_back(cv::Point(keylines[j].getStartPoint().x, keylines[j].getStartPoint().y));
                fit1.push_back(cv::Point(keylines[j].getEndPoint().x, keylines[j].getEndPoint().y));
                cv::Vec4f line_para; 
                cv::fitLine(fit1, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);
                //进行线段拟合
                for (int k = 0; k < fit1.size(); k++)
                {
                    MaxP = fit1[0]; MinP = fit1[0];
                    for (int j = 0; j < fit1.size() - 1; j++)
                    {		
                        if (fit1[j + 1].x > MaxP.x)
                        {
                            MaxP = fit1[j + 1];
                        }
                        
                        if (fit1[j + 1].x < MinP.x)
                        {
                            MinP = fit1[j + 1];
                        }			
                    }

                }
                //获取点斜式的点和斜率
                cv::Point point0;
                point0.x = line_para[2];
                point0.y = line_para[3];
                double k = line_para[1] / line_para[0];

                //计算直线的端点(y = k(x - x0) + y0)
                cv::Point point1, point2;
                point1.x =MinP.x;
                point1.y = k * (MinP.x - point0.x) + point0.y;

                point2.x = MaxP.x;
                point2.y = k * (MaxP.x - point0.x) + point0.y;

                keylines[i].startPointX=point1.x;
                keylines[i].startPointY=point1.y;
                keylines[i].endPointX=point2.x;
                keylines[i].endPointY=point2.y;

                keylines[i].sPointInOctaveX=keylines[i].startPointX;
                keylines[i].sPointInOctaveY=keylines[i].startPointY;
                keylines[i].ePointInOctaveX=keylines[i].endPointX;
                keylines[i].ePointInOctaveY=keylines[i].endPointY;

                keylines[i].lineLength=sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y));

                keylines[i].response=-1;


            }


        }
   
    }

    sort(keylines.begin(), keylines.end(),sort_lines_by_response());
    int Cut;
    for(Cut=0; Cut<keylines.size(); Cut++)
    {
        if(keylines[Cut].response<0)
        {
            break;
        }

    }
    keylines.resize(Cut+1);



}



void LineSegment::CreatneLine(const Mat &img,vector<cv::line_descriptor::KeyLine> &Inkeylines,vector<cv::line_descriptor::KeyLine> &Outkeylines,int _line)
{
    if(Inkeylines.empty()) return;

    cv::Mat OrginImage;
    shared_ptr<LineSegment>linelsd;
    OrginImage.create(img.rows,img.cols,CV_8UC3);
    OrginImage.setTo(255);
    
    for(int i=0; i<Inkeylines.size(); i++)
          cv::line(OrginImage,Inkeylines[i].getStartPoint(),Inkeylines[i].getEndPoint(),cv::Scalar (0, 0, 0),1);
    Ptr<cv::line_descriptor::LSDDetector> lsd = cv::line_descriptor::LSDDetector::createLSDDetector();
    lsd->detect(OrginImage, Outkeylines, 1.2, 1);
    if(Outkeylines.size()>200) return;
    sort(Outkeylines.begin(),Outkeylines.end(),sort_lines_by_response());
    if(Outkeylines.size()>_line) Outkeylines.resize(_line);

    // 对比较近的线进行删除
    sort(Outkeylines.begin(),Outkeylines.end(),sort_lines_by_length());
    for(int i=0; i<Outkeylines.size()-1; i=i+2)
    {   
//        float len=abs(Outkeylines[i].angle-Outkeylines[i+1].angle);
//
//        if(len<0.1f)
//        // float distance1=(Outkeylines[i].startPointX-Outkeylines[i+1].startPointX)*(Outkeylines[i].startPointX-Outkeylines[i+1].startPointX)
//        //                 -(Outkeylines[i].startPointY-Outkeylines[i+1].startPointY)*(Outkeylines[i].startPointY-Outkeylines[i+1].startPointY);
//        // float distance2=(Outkeylines[i].endPointX-Outkeylines[i+1].endPointX)*(Outkeylines[i].endPointX-Outkeylines[i+1].endPointX)
//        //                 -(Outkeylines[i].endPointY-Outkeylines[i+1].endPointY)*(Outkeylines[i].endPointY-Outkeylines[i+1].endPointY);
//        // if(distance1<=1.0f||distance2<=0.6f)
        {
            Outkeylines[i].response=-1;

        }
        
    }
    sort(Outkeylines.begin(),Outkeylines.end(),sort_lines_by_response());
    int count=0;
    for(count=Outkeylines.size();count>0;--count)
        if(Outkeylines[count].response!=-1) break;

    Outkeylines.resize(count+1);
//    Outkeylines.resize((Outkeylines.size()-count));
}






}