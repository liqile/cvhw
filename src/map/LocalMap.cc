#include "LocalMap.h"
#include "Pose.h"
#include "ORBmatcher.h"
#include "drawers.h"
#include "display.h"
#include "Map.h"

namespace lqlslam {

void LocalMap::triangularPoints(Frame* f1, Frame* f2) {
    const vector<int>& matchOfF1 = mappingMatcherCounter.lastIdx;
    //const vector<int>& matchOfF2 = mappingMatcherCounter.currIdx;
    int tot = 0;
    int num = 0;
    for (int i = 0; i < f1->features->keyPointsNum; i++) {
        int idx2 = matchOfF1[i];
        if (idx2 == -1) {
            continue;
        }
        tot ++;
        const Feature& ft1 = f1->features->keyPoints[i];
        const Feature& ft2 = f2->features->keyPoints[idx2];
        cv::Mat pos = Pose::triangular(f1->pose, f2->pose, ft1.keyPoint, ft2.keyPoint);
        if (pos.empty()) {
            continue;
        }
        //todo add new point
        MapPoint* p = new MapPoint(pos, f1, i);
        p->addObservation(f2, idx2);
        num ++;
    }
    cout << "[local map] tot triangular matches: " << tot << endl;
    cout << "[local map] tot map point: " << num << endl;
}

void LocalMap::createMapPoints(Frame *keyFrame) {
    //vector<Frame*> neighbor;
    //getNeighborKeyFrames(keyFrame, neighbor);
    ORBmatcher matcher(keyFrame);
    for (int i = 0; i < neighbor.size(); i++) {
        Frame* neigh = neighbor[i];
        cout << "[local map] triangular : " << keyFrame->frameId << " with " << neigh->frameId << endl;
        matcher.searchByTriangular(neigh);
        triangularPoints(keyFrame, neigh);
        if (displayer->show()) {
            cout << "[local map] triangular debug" << endl;
            matcher.debugTriangular ();
        }
    }
}

void LocalMap::fuseMatches(Frame *f1, Frame *f2) {
    for (int i = 0; i < mappingMatcherCounter.lastIdx.size(); i++) {
        int i2 = mappingMatcherCounter.lastIdx[i];
        if (i2 == -1) {
            continue;
        }
        Feature& fe1 = f1->features->keyPoints[i];
        Feature& fe2 = f2->features->keyPoints[i2];
        if (fe2.mapPoint == NULL) {
            continue;
        }
        if (fe1.mapPoint == NULL) {
            fe1.mapPoint = fe2.mapPoint;
        } else {
            MapPoint* p = MapPoint::merge(fe1.mapPoint, fe2.mapPoint);
            p->trackLog.setMapMatchedFrame(f1->frameId);
        }
    }
}

void LocalMap::fuseMapPoints(Frame* keyFrame) {
    ORBmatcher matcher(keyFrame);
    for (int i = 0; i < neighbor.size(); i++) {
        Frame* f2 = neighbor[i];
        matcher.searchForFuse(f2, 2);
        fuseMatches(keyFrame, f2);
    }
    for (int i = 0; i < neighbor.size(); i++) {
        Frame* f1 = neighbor[i];
        ORBmatcher m(f1);
        for (int j = 0; j < keyFrame->features->keyPointsNum; j++) {
            Feature& f = keyFrame->features->keyPoints[j];
            if (f.mapPoint == NULL) {
                continue;
            }
            f.mapPoint->trackLog.setMapMatchedFrame(-1);
        }
        m.searchForFuse(keyFrame, 2);
        fuseMatches(f1, keyFrame);
    }
    for (int i = 0; i < keyFrame->features->keyPointsNum; i++) {
        Feature& f = keyFrame->features->keyPoints[i];
        if (f.mapPoint == NULL) {
            continue;
        }
        f.mapPoint->observation.updateDescriptor();
    }
}

void LocalMap::processTrackedPoints(Frame *keyFrame) {
    for (int i = 0; i < keyFrame->features->keyPointsNum; i++) {
        Feature& f = keyFrame->features->keyPoints[i];
        if (f.mapPoint == NULL) {
            continue;
        }
        f.mapPoint->addObservation(keyFrame, i);
        f.mapPoint->observation.updateDescriptor();
    }
}

void LocalMap::cullMapPoint(Frame *keyFrame) {
    list<MapPoint*>::iterator itr = newMapPoints.begin();
    while (itr != newMapPoints.end()) {
        MapPoint* p = *itr;
        if (!p->good) {
            itr = newMapPoints.erase(itr);
            continue;
        }
        if (keyFrame->keyFrameId - p->observation.firstKFId() >= 2 && p->observation.observations.size() <= 2) {
            p->destroy();
            itr = newMapPoints.erase(itr);
            continue;
        }
        if (keyFrame->keyFrameId - p->observation.firstKFId() >= 3) {
            itr = newMapPoints.erase(itr);
            continue;
        }
        itr ++;
    }
}

void LocalMap::processNewKeyFrame(Frame *keyFrame) {
    processTrackedPoints(keyFrame);
    cullMapPoint(keyFrame);
    neighbor.clear();
    getNeighborKeyFrames(keyFrame, neighbor);
    cout << "[local map] key frame [frameid]: " << keyFrame->frameId << endl;
    for (int i = 0; i < neighbor.size(); i++) {
        cout << neighbor[i]->frameId << " ";
    }
    cout << endl;
    createMapPoints(keyFrame);
    keyFrames.push_back (keyFrame);
}

LocalMap::LocalMap() {
    clear();
}

void LocalMap::initialize(Frame* keyFrame) {
    newMapPoints.clear();
    for (int i = 0; i < keyFrame->features->keyPointsNum; i++) {
        Feature& f = keyFrame->features->keyPoints[i];
        newMapPoints.push_back(f.mapPoint);
    }
}

void LocalMap::clear() {
    keyFrames.clear();
}

void LocalMap::addKeyFrame(Frame* keyFrame) {
    processNewKeyFrame(keyFrame);
}

void LocalMap::getNeighborKeyFrames(const Frame* frame, std::vector<Frame*>& frames) {
    frames.clear();
#if 0
    for (int i = 0; i < keyFrames.size(); i++) {
        Frame* f = keyFrames[i];
        //if (frame->keyFrameId - f->keyFrameId <= 3) {
        if (f->frameId == 0) {
            frames.push_back(f);
        }
    }
#endif
    mapInfo.getNeighborKF(frame, frames);
}

}
