#include "Svd.h"

namespace lqlslam {

CameraParam* Svd::getCameraParam() {
    return camera;
}

cv::Mat Svd::getK() {
    CameraParam* c = getCameraParam();
    return c->k;
}

void Svd::getAllPoses(const cv::Mat &F, cv::Mat &R1, cv::Mat &R2, cv::Mat &t1, cv::Mat &t2) {
    cout << "svd for all poses" << endl;
    cv::Mat K = getK();
    cout << "F = " << F << endl;
    cout << "K = " << K << endl;
    cout << "K.type: " << K.type() << endl;
    cout << "F.type: " << F.type() << endl;
    cv::Mat E = K.t() * F * K;
    cout << "E = " << E << endl;
    cv::Mat u, w, vt, t;
    cv::SVD::compute (E, w, u, vt);
    u.col(2).copyTo(t);
    t = t / cv::norm(t);
    cv::Mat Rz(3, 3, CV_32F, cv::Scalar(0));
    Rz.at<float>(0, 1) = -1;
    Rz.at<float>(1, 0) = 1;
    Rz.at<float>(2, 2) = 1;
    R1 = u * Rz * vt;
    R2 = u * Rz.t() * vt;
    if (cv::determinant(R1) < 0) {
        R1 = - R1;
        R2 = - R2;
    }
    t1 = t;
    t2 = -t;
}

void Svd::getDepth (const Pose& p2, const vector<cv::KeyPoint> &kp1, const vector<cv::KeyPoint> &kp2, vector<cv::Mat>& points) {
    points.clear();

    cv::Mat I(4, 4, CV_32F);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (i == j) {
                I.at<float>(i, j) = 1;
            } else {
                I.at<float>(i, j) = 0;
            }
        }
    }
    Pose p1;
    p1.setPose(I);

    for (int i = 0; i < kp1.size(); i++) {
        const cv::KeyPoint& kpt1 = kp1[i];
        const cv::KeyPoint& kpt2 = kp2[i];
        cv::Mat p = Pose::triangular(p1, p2, kpt1, kpt2);
        points.push_back(p);
    }
}

int Svd::countPositive (const Pose& pose, const vector<cv::Mat>& points) {
    int n = points.size();
    int count = 0;
    int negCount = 0;
    for (int i = 0; i < n; i++) {
        const cv::Mat& p = points[i];
        if (p.empty ()) {
            continue;
        }
        if (p.at<float>(2, 0) <= 0) {
            negCount ++;
            continue;
        }
        cv::Mat p2 = pose.toLocal(p);
        if (p2.at<float>(2, 0) <= 0) {
            negCount ++;
            continue;
        }
        count ++;
    }
    cout << count << " " << negCount << " " << points.size() << endl;
    return count;
}

cv::Mat Svd::getFundamental(const vector<cv::Point2f> &p1, const vector<cv::Point2f> &p2, vector<bool> &mask) {
    int n = p1.size();
    mask.clear();
    for (int i = 0; i < n; i++) {
        mask.push_back(false);
    }
    cv::Mat maskMat;
    cv::Mat fundamental = cv::findFundamentalMat (p1, p2, maskMat);
    for (int i = 0; i < n; i++) {
        if (maskMat.at<uchar>(i, 0) != 0) {
            mask[i] = true;
        }
    }
    return fundamental;
}

bool Svd::getRt(const vector<cv::KeyPoint> &kp1, const vector<cv::KeyPoint> &kp2, const cv::Mat &F, Pose& pose) {
    cout << "svd for R and t" << endl;
    cv::Mat R1, R2, t1, t2;
    getAllPoses(F, R1, R2, t1, t2);
    vector<cv::Mat> points11;
    vector<cv::Mat> points12;
    vector<cv::Mat> points21;
    vector<cv::Mat> points22;
    Pose pose2_11;
    Pose pose2_12;
    Pose pose2_21;
    Pose pose2_22;
    pose2_11.setPose (R1, t1);
    pose2_12.setPose (R1, t2);
    pose2_21.setPose (R2, t1);
    pose2_22.setPose (R2, t2);
    getDepth (pose2_11, kp1, kp2, points11);
    getDepth (pose2_12, kp1, kp2, points12);
    getDepth (pose2_21, kp1, kp2, points21);
    getDepth (pose2_22, kp1, kp2, points22);
    int count11 = countPositive(pose2_11, points11);
    int count12 = countPositive(pose2_12, points12);
    int count21 = countPositive(pose2_21, points21);
    int count22 = countPositive(pose2_22, points22);
    int maxCount = max(count11, count12);
    maxCount = max(maxCount, count21);
    maxCount = max(maxCount, count22);
    int good = 0;
    if (count11 > maxCount * 0.5) {
        good ++;
    }
    if (count12 > maxCount * 0.5) {
        good ++;
    }
    if (count21 > maxCount * 0.5) {
        good ++;
    }
    if (count22 > maxCount * 0.5) {
        good ++;
    }
    if (good != 1) {
        return false;
    } else {
        if (count11 == maxCount) {
            pose = pose2_11;
        } else if (count12 == maxCount) {
            pose = pose2_12;
        } else if (count21 == maxCount) {
            pose = pose2_21;
        } else {
            pose = pose2_22;
        }
        cout << "pose of frame2: " << pose.mTcw << endl;
        return true;
    }
}

}
