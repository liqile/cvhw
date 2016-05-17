
#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>
#include"Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include"Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace lqlslam
{

class Converter {
public:
    static g2o::SE3Quat mat2SE3Quat(const cv::Mat& cvT);
    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    static cv::Mat toCvMat(const g2o::Vector3d& v);
    /*
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
    */
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    static Eigen::Isometry3d toEigenT(const cv::Mat& mTcw);
};
}

#endif // CONVERTER_H
