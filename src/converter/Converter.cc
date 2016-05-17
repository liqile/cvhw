
#include "Converter.h"

namespace lqlslam
{

g2o::SE3Quat Converter::mat2SE3Quat(const cv::Mat& cvT) {
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
         cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
         cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);
    Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));
    return g2o::SE3Quat(R,t);
}
cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3) {
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    return toCvMat(eigMat);
}
cv::Mat Converter::toCvMat(const g2o::Vector3d& v) {
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++) {
        cvMat.at<float>(i)=v(i);
    }
    return cvMat.clone();
}

/*
cv::Mat Converter::toCvMat(const g2o::Sim3 &Sim3)
{
    Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
    Eigen::Vector3d eigt = Sim3.translation();
    double s = Sim3.scale();
    return toCvSE3(s*eigR,eigt);
}
*/
cv::Mat Converter::toCvMat(const Eigen::Matrix<double,4,4> &m) {
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

Eigen::Isometry3d Converter::toEigenT(const cv::Mat& mTcw) {
    Eigen::Isometry3d pose;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            pose(i, j) = mTcw.at<float>(i, j);
        }
    }
    return pose;
}

} //namespace ORB_SLAM
