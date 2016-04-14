#include "Pose.h"
#include "parameter.h"

namespace lqlslam {

void Pose::setPose(const cv::Mat& mTcw) {
    this->mTcw = mTcw.clone ();
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}
cv::Mat Pose::toWorld(const cv::Mat& localPos) const {
    return mRwc * localPos + mOw;
}
cv::Mat Pose::toLocal(const cv::Mat& worldPos) const {
    return mRwc.t() * (worldPos - mOw);
}
void Pose::computeEpipole(const Pose& p1, float& ex, float& ey) {
    const cv::Mat& c1 = p1.mOw;
    const cv::Mat& c12 = toLocal(c1);
    camera->project(c12, ex, ey);
}
cv::Mat Pose::getF12(const Pose& p1) {
    const cv::Mat& rot1 = p1.mRcw;
    const cv::Mat& t1 = p1.mtcw;
    const cv::Mat& rot2 = mRcw;
    const cv::Mat& t2 = mtcw;
    cv::Mat rot12 = rot1 * rot2.t();
    cv::Mat t12 = -rot12 * t2 + t1;
    cv::Mat t12x = Pose::skewSymmetricMatrix(t12);
    const cv::Mat &k = camera->k;
    return k.t().inv() * t12x * rot12 * k.inv();
}

cv::Mat Pose::skewSymmetricMatrix (const cv::Mat &v) {
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}
bool Pose::checkDistEpipolarLine(const cv::KeyPoint &kp1,const cv::KeyPoint &kp2,const cv::Mat &F12) {
    // Epipolar line in second image l = x1'F12 = [a b c]
    const float a = kp1.pt.x*F12.at<float>(0,0)+kp1.pt.y*F12.at<float>(1,0)+F12.at<float>(2,0);
    const float b = kp1.pt.x*F12.at<float>(0,1)+kp1.pt.y*F12.at<float>(1,1)+F12.at<float>(2,1);
    const float c = kp1.pt.x*F12.at<float>(0,2)+kp1.pt.y*F12.at<float>(1,2)+F12.at<float>(2,2);

    const float num = a*kp2.pt.x+b*kp2.pt.y+c;

    const float den = a*a+b*b;

    if(den==0)
        return false;

    const float dsqr = num*num/den;

    return dsqr<3.84*extract->levelSigma2[kp2.octave];
}

}
