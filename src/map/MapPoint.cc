#include "MapPoint.h"
#include "ORBmatcher.h"

namespace lqlslam {

Observation::Observation() {
    needUpdate = false;
    refKF = NULL;
    observations.clear();
}

void Observation::updateDescriptor() {
    if (needUpdate) {
        return;
    }
    if (observations.empty()) {
        return;
    }
    //todo
    vector<cv::Mat> vDes;
    map<Frame*, int>::iterator itr = observations.begin();
    while (itr != observations.end()) {
        Frame* f = itr->first;
        vDes.push_back(f->features->descriptors.row(itr->second));
        itr ++;
    }
    const int N = vDes.size();
    float dis[N][N];
    for (int i = 0; i < N; i++) {
        dis[i][i] = 0;
        for (int j = i + 1; j < N; j++) {
            int d = ORBmatcher::descriptorDis(vDes[i], vDes[j]);
            dis[i][j] = d;
            dis[j][i] = d;
        }
    }
    int bestMedian = 1000;
    int bestIdx = 0;
    for (int i = 0; i < N; i++) {
        vector<int> v(dis[i], dis[i] + N);
        sort(v.begin(), v.end());
        int m = v[0.5 * (N - 1)];
        if (m < bestMedian) {
            bestMedian = m;
            bestIdx = i;
        }
    }
    descriptor = vDes[bestIdx].clone();
    needUpdate = true;
}

void Observation::addObservation(const cv::Mat& pos, Frame* keyFrame, int index) {
    observations[keyFrame] = index;
    if (refKF == NULL) {
        refKF = keyFrame;
        descriptor = keyFrame->features->descriptors.row(index);
        const cv::KeyPoint& kpr = keyFrame->features->rawKeyPoints[index];
        int level = kpr.octave;
        float dis = keyFrame->pose.distance(pos);
        float levelScale = extract->scaleFactor[level];
        int restLevel = extract->nLevels - 1 - level;
        float restScale = extract->scaleFactor[restLevel];
        minDis = dis / levelScale;
        maxDis = dis * restScale;
    }
    if (observations.size() <= 2) {
        needUpdate = false;
    } else {
        needUpdate = false;
    }
}

int Observation::decObservation(Frame *keyFrame) {
    if (observations.count(keyFrame) > 0) {
        observations.erase(keyFrame);
    }
    if (refKF == keyFrame) {
        refKF = observations.begin()->first;
    }
    return observations.size();
}

int Observation::firstKFId() {
    return refKF->keyFrameId;
}

void Marker::setTrackMatchedFrame(int id) {
    trackMatchedFrame = id;
}

void Marker::setMapMatchedFrame(int id) {
    mapMatchedFrame = id;
}

Marker::Marker() {
    setTrackMatchedFrame(-1);
    setMapMatchedFrame(-1);
}

int MapPoint::nextId = 0;

MapPoint::MapPoint(const cv::Mat& pos, Frame* keyFrame, int index) {
    good = true;
    pointId = nextId;
    nextId ++;
    trackLog.setTrackMatchedFrame(-1);
    this->pos = pos.clone();
    addObservation(keyFrame, index);
}

void MapPoint::addObservation(Frame* keyFrame, int index) {
    observation.addObservation(pos, keyFrame, index);
    keyFrame->features->setMapPoint(index, this);
}

void MapPoint::decObservation(Frame* keyFrame) {
    int index = observation.observations[keyFrame];
    int r = observation.decObservation(keyFrame);
    if (r < 2) {
        this->destroy();
    }
    keyFrame->features->eraseMapPoint(index);
}

cv::Mat MapPoint::getCamPos(const Pose& pose) {
    return pose.mRcw * pos + pose.mtcw;
}

void MapPoint::setPointPos(const cv::Mat& pos) {
    this->pos = pos;
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

int MapPoint::getOctave(float dis) {
    //todo
    float rate = dis / observation.minDis;
    return ceil((log(rate) / log(extract->fScaleFactor)));
}

float MapPoint::getMinDis() {
    return observation.minDis / extract->fScaleFactor;
}

float MapPoint::getMaxDis() {
    return observation.maxDis * extract->fScaleFactor;
}

void MapPoint::merge(MapPoint *p) {
    if (p->pointId == pointId) {
        return;
    }
    map<Frame*, int>& o = p->observation.observations;
    map<Frame*, int>::iterator itr = o.begin();
    while (itr != o.end()) {
        Frame* f = itr->first;
        if (observation.observations.count(f) != 0) {
            f->features->eraseMapPoint(itr->second);
        } else {
            addObservation(f, itr->second);
        }
        itr ++;
    }
    observation.updateDescriptor();
}

void MapPoint::destroy() {
    good = false;
}

MapPoint* MapPoint::merge(MapPoint* p1, MapPoint* p2) {
    if (p1->pointId == p2->pointId) {
        return p1;
    }
    int n1 = p1->observation.observations.size();
    int n2 = p2->observation.observations.size();
    if (n1 < n2) {
        p2->merge(p1);
        return p2;
    } else {
        p1->merge(p2);
        return p1;
    }
}

}
