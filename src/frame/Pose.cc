#include "Pose.h"
#include "parameter.h"

namespace lqlslam {

Pose::Pose() {
}

Pose::Pose(const Pose& pose) {
    mTcw = pose.mTcw.clone();
    mRcw = pose.mRcw.clone();
    mtcw = pose.mtcw.clone();
    mRwc = pose.mRwc.clone();
    mOw = pose.mOw.clone();
}

void Pose::setPose(const cv::Mat& mTcw) {
    this->mTcw = mTcw.clone ();
    mRcw = this->mTcw.rowRange(0, 3).colRange(0, 3);
    mRwc = mRcw.t();
    mtcw = this->mTcw.rowRange(0, 3).col(3);
    mOw = -mRcw.t()*mtcw;
}

void Pose::setPose(const cv::Mat& Rcw, const cv::Mat& tcw) {
    this->mTcw = cv::Mat(4, 4, CV_32F);
    mRcw = Rcw.clone();
    mRwc = mRcw.t();
    mtcw = tcw.clone();
    mOw = -mRcw.t()*mtcw;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            this->mTcw.at<float>(i, j) = mRcw.at<float>(i, j);
        }
        this->mTcw.at<float>(i, 3) = mtcw.at<float>(i, 0);
        this->mTcw.at<float>(3, i) = 0;
    }
    this->mTcw.at<float>(3, 3) = 1;
}

cv::Mat Pose::toWorld(const cv::Mat& localPos) const {
    return mRwc * localPos + mOw;
}
cv::Mat Pose::toLocal(const cv::Mat& worldPos) const {
    return mRwc.t() * (worldPos - mOw);
}
void Pose::computeEpipole(const Pose& p1, float& ex, float& ey) const {
    const cv::Mat& c1 = p1.mOw;
    const cv::Mat& c12 = toLocal(c1);
    camera->project(c12, ex, ey);
}
cv::Mat Pose::getF12(const Pose& p1) const {
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
bool Pose::checkPoint(const cv::Mat &pos, const cv::KeyPoint& k) const {
    cv::Mat lp = toLocal(pos);
    float z = lp.at<float>(2);
    if (z <= 0) {
        return false;
    }
    float x, y;
    camera->project(lp, x, y);
    float dx = x - k.pt.x;
    float dy = y - k.pt.y;
    if (dx * dx + dy * dy > 5.991 * extract->levelSigma2[k.octave]) {
        return false;
    }
    return true;
}
float Pose::distance(const cv::Mat &wp) const {
    return cv::norm(wp - mOw);
}

cv::Mat Pose::skewSymmetricMatrix (const cv::Mat &v) {
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}
void Pose::getEpipolarLine(const cv::KeyPoint &kp1, const cv::Mat &F12, float &a, float &b, float &c) {
    a = kp1.pt.x*F12.at<float>(0,0)+kp1.pt.y*F12.at<float>(1,0)+F12.at<float>(2,0);
    b = kp1.pt.x*F12.at<float>(0,1)+kp1.pt.y*F12.at<float>(1,1)+F12.at<float>(2,1);
    c = kp1.pt.x*F12.at<float>(0,2)+kp1.pt.y*F12.at<float>(1,2)+F12.at<float>(2,2);
}

bool Pose::checkDistEpipolarLine(const cv::KeyPoint &kp1,const cv::KeyPoint &kp2,const cv::Mat &F12) {
    // Epipolar line in second image l = x1'F12 = [a b c]
    const float a = kp1.pt.x*F12.at<float>(0,0)+kp1.pt.y*F12.at<float>(1,0)+F12.at<float>(2,0);
    const float b = kp1.pt.x*F12.at<float>(0,1)+kp1.pt.y*F12.at<float>(1,1)+F12.at<float>(2,1);
    const float c = kp1.pt.x*F12.at<float>(0,2)+kp1.pt.y*F12.at<float>(1,2)+F12.at<float>(2,2);

    const float num = a*kp2.pt.x+b*kp2.pt.y+c;

    const float den = a*a+b*b;

    if(den==0) {
        return false;
    }
    const float dsqr = num*num/den;

    return dsqr<3.84*extract->levelSigma2[kp2.octave];
}

cv::Mat Pose::triangular(const Pose &p1, const Pose &p2, const cv::KeyPoint &k1, const cv::KeyPoint &k2) {
    //todo
    cv::Mat x1 = camera->getRay(k1.pt.x, k1.pt.y);
    cv::Mat ray1 = p1.toWorld(x1);
    cv::Mat x2 = camera->getRay(k2.pt.x, k2.pt.y);
    cv::Mat ray2 = p2.toWorld(x2);
    float cosRay = ray1.dot(ray2) / (cv::norm(ray1) * cv::norm(ray2));
    //cout << "cos ray: " << cosRay << endl;
    if (cosRay > 0 && cosRay < 0.9998) {
        //cout << " enter cos " << endl;
        cv::Mat A(4, 4, CV_32F);
        A.row(0) = x1.at<float>(0) * p1.mTcw.row(2) - p1.mTcw.row(0);
        A.row(1) = x1.at<float>(1) * p1.mTcw.row(2) - p1.mTcw.row(1);
        A.row(2) = x2.at<float>(0) * p2.mTcw.row(2) - p2.mTcw.row(0);
        A.row(3) = x2.at<float>(1) * p2.mTcw.row(2) - p2.mTcw.row(1);
        cv::Mat w,u,vt;
        cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
        cv::Mat pos = vt.row(3).t();
        if (pos.at<float>(3) == 0) {
            return cv::Mat();
        }
        pos = pos.rowRange(0, 3) / pos.at<float>(3);
        pos = pos.rowRange(0, 3);
        if (!p1.checkPoint(pos, k1)) {
            return cv::Mat();
        }
        if (!p2.checkPoint(pos, k2)) {
            return cv::Mat();
        }
        float d1 = p1.distance(pos);
        float d2 = p2.distance(pos);
        float s = 1.5 * extract->scaleFactor[1];
        float rateDistance = d1 / d2;
        float rateOctave = extract->scaleFactor[k1.octave] / extract->scaleFactor[k2.octave];
        if (rateDistance * s < rateOctave || rateDistance > rateOctave * s) {
            return cv::Mat();
        }
        return pos;
    }
    return cv::Mat();
}

cv::Mat Pose::triangularInit(const Pose &p1, const Pose &p2, const cv::KeyPoint &k1, const cv::KeyPoint &k2) {
    //todo
    cv::Mat x1 = camera->getRay(k1.pt.x, k1.pt.y);
    cv::Mat ray1 = p1.toWorld(x1);
    cv::Mat x2 = camera->getRay(k2.pt.x, k2.pt.y);
    cv::Mat ray2 = p2.toWorld(x2);
    float cosRay = ray1.dot(ray2) / (cv::norm(ray1) * cv::norm(ray2));
    //cout << "cos ray: " << cosRay << endl;
    if (cosRay > 0 && cosRay < 0.9998) {
        //cout << " enter cos " << endl;
        cv::Mat A(4, 4, CV_32F);
        A.row(0) = x1.at<float>(0) * p1.mTcw.row(2) - p1.mTcw.row(0);
        A.row(1) = x1.at<float>(1) * p1.mTcw.row(2) - p1.mTcw.row(1);
        A.row(2) = x2.at<float>(0) * p2.mTcw.row(2) - p2.mTcw.row(0);
        A.row(3) = x2.at<float>(1) * p2.mTcw.row(2) - p2.mTcw.row(1);
        cv::Mat w,u,vt;
        cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
        cv::Mat pos = vt.row(3).t();
        if (pos.at<float>(3) == 0) {
            return cv::Mat();
        }
        pos = pos.rowRange(0, 3) / pos.at<float>(3);
        pos = pos.rowRange(0, 3);
        return pos;
    }
    return cv::Mat();
}

}
