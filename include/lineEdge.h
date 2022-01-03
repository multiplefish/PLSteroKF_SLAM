
#ifndef ORB_SLAM2_LINEEDGE_H
#define ORB_SLAM2_LINEEDGE_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace types_six_dof_expmap {
    void init();
}
// 含有右边的值
class EdgeLineProjectXYZOnlyPose : public g2o::BaseUnaryEdge<4, Eigen::Vector4d, g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeLineProjectXYZOnlyPose() {}

    virtual void computeError()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector4d obs = _measurement;
        Eigen::Vector3d proj = cam_project(v1->estimate().map(Xw));
        double A=1/sqrt(obs(0) *obs(0) +obs(1) *obs(1));
        _error(0) = (obs(0) * proj(0) + obs(1) * proj(1) + obs(2))*A;
        _error(1) = (obs(0) * (obs(3)+proj(2)) + obs(1) * proj(1) + obs(2))*A;
        _error(2) = 0;
        _error(3) = 0;
        
    }
    
    bool isDepthPositive()
    {
    const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
    return (v1->estimate().map(Xw))(2)>=0.0&&(v1->estimate().map(Xw))(5)>=0.0;
    }
    double chiline() {
        return _error(0) * _error(0)*0.5+_error(1) * _error(1)*0.5f;
    }

    virtual void linearizeOplus()
    {

        g2o::VertexSE3Expmap *vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d xyz_trans = vi->estimate().map(Xw);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0 / xyz_trans[2];
        double invz_2 = invz * invz;

        double lx = _measurement(0);
        double ly = _measurement(1);
        double invsqrtlx2ly2=1/sqrt(lx*lx+ly*ly);
        double invz_2sqrtlx2ly2=1/(sqrt(lx*lx+ly*ly)*invz_2);
        double invzsqrtlx2ly2=1/(sqrt(lx*lx+ly*ly)*invz);

        _jacobianOplusXi(0, 0) = fx*ly*invsqrtlx2ly2+(y*(x*fx*lx+y*fy*ly))*invz_2sqrtlx2ly2; 
        _jacobianOplusXi(0, 1) = -(fx*lx)*invsqrtlx2ly2-(x*(x*fx*lx+y*fy*ly))*invz_2sqrtlx2ly2;
        _jacobianOplusXi(0, 2) = (y*fx*lx)*invzsqrtlx2ly2-(x*fy*ly)*invzsqrtlx2ly2;

        _jacobianOplusXi(0, 3) = -(fx*lx)*invzsqrtlx2ly2;
        _jacobianOplusXi(0, 4) = -(fy*ly)*invzsqrtlx2ly2;
        _jacobianOplusXi(0, 5) = (x*fx*lx+y*fy*ly)*invz_2sqrtlx2ly2;


        _jacobianOplusXi(1, 0) = y*((bf*lx)*invz_2sqrtlx2ly2+(y*fy*ly)*invz_2sqrtlx2ly2)+(fy*ly*invsqrtlx2ly2);
        _jacobianOplusXi(1, 1) = -x*((bf*lx)*invz_2sqrtlx2ly2+(y*fy*ly)*invz_2sqrtlx2ly2);
        _jacobianOplusXi(1, 2) = -(x*fy*ly)*invzsqrtlx2ly2;
        _jacobianOplusXi(1, 3) = 0;
        _jacobianOplusXi(1, 4) = -(fy*ly)*invzsqrtlx2ly2;
        _jacobianOplusXi(1, 5) = (bf*lx)*invz_2sqrtlx2ly2+(y*fy*ly)*invz_2sqrtlx2ly2;

        _jacobianOplusXi(2, 0) = 0;
        _jacobianOplusXi(2, 1) = 0;
        _jacobianOplusXi(2, 2) = 0;
        _jacobianOplusXi(2, 3) = 0;
        _jacobianOplusXi(2, 4) = 0;
        _jacobianOplusXi(2, 5) = 0;
    }

    Eigen::Vector3d cam_project(const Eigen::Vector3d &trans_xyz) {
        const float invz = 1.0f/trans_xyz[2];
        Eigen::Vector2d proj = g2o::project(trans_xyz);
        
        Eigen::Vector3d res;
        res[0] = proj[0] * fx + cx;
        res[1] = proj[1] * fy + cy;
        res[2] = bf*invz;
        return res;
    }
    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}
    Eigen::Vector3d Xw;
    double fx, fy, cx, cy ,bf,err0,err1;
    bool bad0,bad1;
};

class EdgeLineProjectXYZ: public g2o::BaseBinaryEdge<4, Eigen::Vector4d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeLineProjectXYZ(){}


    virtual void computeError()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);

        Eigen::Vector4d obs = _measurement;
        Eigen::Vector3d proj = cam_project(v1->estimate().map(v2->estimate()));

         double A=1/sqrt(obs(0) *obs(0) +obs(1) *obs(1));
        _error(0) = (obs(0) * proj(0) + obs(1) * proj(1) + obs(2))*A;
        _error(1) = (obs(0) * (obs(3)+proj(2)) + obs(1) * proj(1) + obs(2))*A;
        _error(2) = 0;
        _error(3) = 0;
       
    }

    virtual void linearizeOplus()
    {
        // 位姿
        const g2o::VertexSE3Expmap *vj = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);

        // 空间点
        const g2o::VertexSBAPointXYZ *vi = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        g2o::SE3Quat T(vj->estimate());
        Eigen::Vector3d xyz = vi->estimate();
        Eigen::Vector3d xyz_trans = T.map(xyz);

        const Matrix3d R =  T.rotation().toRotationMatrix();

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0 / xyz_trans[2];
        double invz_2 = invz * invz;

        double lx = _measurement(0);
        double ly = _measurement(1);
        double invsqrtlx2ly2=1/sqrt(lx*lx+ly*ly);
        double invz_2sqrtlx2ly2=1/(sqrt(lx*lx+ly*ly)*invz_2);
        double invzsqrtlx2ly2=1/(sqrt(lx*lx+ly*ly)*invz);

        
        _jacobianOplusXj(0, 0) = fx*ly*invsqrtlx2ly2+(y*(x*fx*lx+y*fy*ly))*invz_2sqrtlx2ly2; 
        _jacobianOplusXj(0, 1) = -(fx*lx)*invsqrtlx2ly2-(x*(x*fx*lx+y*fy*ly))*invz_2sqrtlx2ly2;
        _jacobianOplusXi(0, 2) = (y*fx*lx)*invzsqrtlx2ly2-(x*fy*ly)*invzsqrtlx2ly2;

        _jacobianOplusXj(0, 3) = -(fx*lx)*invzsqrtlx2ly2;
        _jacobianOplusXj(0, 4) = -(fy*ly)*invzsqrtlx2ly2;
        _jacobianOplusXj(0, 5) = (x*fx*lx+y*fy*ly)*invz_2sqrtlx2ly2;


        _jacobianOplusXj(1, 0) = y*((bf*lx)*invz_2sqrtlx2ly2+(y*fy*ly)*invz_2sqrtlx2ly2)+(fy*ly*invsqrtlx2ly2);
        _jacobianOplusXj(1, 1) = -x*((bf*lx)*invz_2sqrtlx2ly2+(y*fy*ly)*invz_2sqrtlx2ly2);
        _jacobianOplusXj(1, 2) = -(x*fy*ly)*invzsqrtlx2ly2;
        _jacobianOplusXj(1, 3) = 0;
        _jacobianOplusXj(1, 4) = -(fy*ly)*invzsqrtlx2ly2;
        _jacobianOplusXj(1, 5) = (bf*lx)*invz_2sqrtlx2ly2+(y*fy*ly)*invz_2sqrtlx2ly2;

        _jacobianOplusXj(2, 0) = 0;
        _jacobianOplusXj(2, 1) = 0;
        _jacobianOplusXj(2, 2) = 0;
        _jacobianOplusXj(2, 3) = 0;
        _jacobianOplusXj(2, 4) = 0;
        _jacobianOplusXj(2, 5) = 0;

        // 需要matlab计算
        _jacobianOplusXi(0,0) = (R(2,0)*(x*fx*lx+y*fy*ly))*invz_2sqrtlx2ly2-(R(0,0)*fx*lx)*invzsqrtlx2ly2-(R(1,0)*fy*ly*invzsqrtlx2ly2);
        _jacobianOplusXi(0,1) = (R(2,1)*(x*fx*lx+y*fy*ly))*invz_2sqrtlx2ly2-(R(0,1)*fx*ly)*invzsqrtlx2ly2-(R(1,1)*fy*ly)*invzsqrtlx2ly2;
        _jacobianOplusXi(0,2) = (R(2,2)*(x*fx*lx+y*fy*ly)*invz_2sqrtlx2ly2-(R(0,2)*fx*lx)*invzsqrtlx2ly2-(R(1,2)*fy*ly)*invzsqrtlx2ly2);

        _jacobianOplusXi(1,0) = R(2,0)*((bf*lx)*invz_2sqrtlx2ly2)+(y*fy*ly)*invz_2sqrtlx2ly2-(R(1,0)*fy*ly)*invzsqrtlx2ly2;
        _jacobianOplusXi(1,1) = R(2,1)*((bf*lx)*invz_2sqrtlx2ly2)+(y*fy*ly)*invz_2sqrtlx2ly2-(R(1,1)*fy*ly)*invzsqrtlx2ly2;
        _jacobianOplusXi(1,2) = R(2,2)*((bf*lx)*invz_2sqrtlx2ly2)+(y*fy*ly)*invz_2sqrtlx2ly2-(R(1,2)*fy*ly)*invzsqrtlx2ly2;

        _jacobianOplusXi(2,0) = 0;
        _jacobianOplusXi(2,1) = 0;
        _jacobianOplusXi(2,2) = 0;
    }
    double chiline()
    {
        return _error(0) * _error(0)*0.5+_error(1) * _error(1)*0.5f;
    }
    bool isDepthPositive()
    {
        const g2o::VertexSE3Expmap * v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        return (v1->estimate().map(v2->estimate()))(2)>=0.0&&(v1->estimate().map(v2->estimate()))(5)>=0.0;
    }


    Eigen::Vector3d cam_project(const Eigen::Vector3d &trans_xyz)
    {
        const float invz = 1.0f/trans_xyz[2];
        Eigen::Vector2d proj = g2o::project(trans_xyz);
        Eigen::Vector3d res;
        res[0] = proj[0] * fx + cx;
        res[1] = proj[1] * fy + cy;
        res[2] = bf*invz;
        return res;
    }
    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}
    double fx, fy, cx, cy,bf,err0,err1;
     bool bad0,bad1;
};




#endif //ORB_SLAM2_LINEEDGE_H
