#include "Pose.h"

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
}
