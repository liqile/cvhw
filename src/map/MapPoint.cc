#include "MapPoint.h"

namespace lqlslam {

Observation::Observation() {
    refKF = NULL;
    observations.clear();
}

MapPoint::MapPoint(const cv::Mat& pos, Frame* keyFrame, int index) {
    this->pos = pos.clone();
    addObservation(keyFrame, index);
}

void MapPoint::addObservation(Frame* keyFrame, int index) {
    observation.observations[keyFrame] = index;
    if (observation.refKF == NULL) {
        observation.refKF = keyFrame;
        observation.descriptor = keyFrame->features->descriptors.row(index);
    }
}

cv::Mat MapPoint::getCamPos(const Pose& pose) {
    return pose.mRcw * pos + pose.mtcw;
}

bool MapPoint::reProject (const Pose &pose, float &u, float &v, float &invz, float &ur) {
        cv::Mat p = getCamPos (pose);
        float xc = p.at<float>(0);
        float yc = p.at<float>(1);
        invz = 1.0/p.at<float>(2);
        if (invz < 0) {
            return false;
        }
        u = camera->fx*xc*invz+camera->cx;
        if (u < camera->minX || u > camera->maxX) {
            return false;
        }
        v = camera->fy*yc*invz+camera->cy;
        if (v < camera->minY || v > camera->maxY) {
            return false;
        }
        ur = u - camera->basef * invz;
        return true;
}

cv::Mat MapPoint::getDiscriptor() {
    assert(!observation.descriptor.empty ());
    return observation.descriptor;
}

}
