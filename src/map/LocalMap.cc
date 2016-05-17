#include "LocalMap.h"
#include "Pose.h"
#include "ORBmatcher.h"
#include "drawers.h"
#include "display.h"
#include "Map.h"
#include "Optimizer.h"
#include <set>

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
            mappingMatcherCounter.reduceMatch(i);
            continue;
        }
        //todo add new point
        MapPoint* p = new MapPoint(pos, f1, i);
        p->addObservation(f2, idx2);
        num ++;
        newMapPoints.push_back(p);
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
#if DEBUG_TRIANGULAR_MATCH
        mappingMatcherDrawer.drawMatches(keyFrame, neigh, mappingMatcherCounter.lastIdx);
        if (displayer->show()) {
            cout << "[local map] triangular debug" << endl;
            //matcher.debugTriangular ();
            cv::Mat f12 = matcher.getFun12();
            mappingMatcherDrawer.drawMatchOneByOne(keyFrame, neigh, f12, mappingMatcherCounter.lastIdx);
        }
#endif
    }
}

void LocalMap::fuseMatches(Frame *f1, Frame *f2) {
    cout << "[local mapping] fuse by project " << f2->frameId << " to " << f1->frameId << endl;
    int countNull = 0;
    int countFuse = 0;
    for (int i = 0; i < mappingMatcherCounter.lastIdx.size(); i++) {
        int i2 = mappingMatcherCounter.lastIdx[i];
        if (i2 == -1) {
            continue;
        }
        Feature& fe1 = f1->features->keyPoints[i];
        Feature& fe2 = f2->features->keyPoints[i2];
        if (fe2.mapPoint == NULL || !fe2.mapPoint->good) {
            continue;
        }
        if (fe1.mapPoint == NULL || !fe1.mapPoint->good) {
            countNull ++;
            fe1.mapPoint = fe2.mapPoint;
            mappingMatcherCounter.reduceMatch(i);
        } else {
            if (fe1.mapPoint->pointId != fe2.mapPoint->pointId) {
                MapPoint* p = MapPoint::merge(fe1.mapPoint, fe2.mapPoint);
                p->trackLog.setMapMatchedFrame(f1->frameId);
                countFuse ++;
            } else {
                mappingMatcherCounter.reduceMatch(i);
            }
        }
    }
    cout << "[local mapping] fill hole " << countNull << endl;
    cout << "[local mapping] fuse " << countFuse << endl;
    mappingMatcherDrawer.drawMatches(f1, f2, mappingMatcherCounter.lastIdx);
    displayer->show();
}

void LocalMap::fuseMapPoints(Frame* keyFrame) {
    ORBmatcher matcher(keyFrame);
    for (int i = 0; i < neighbor.size(); i++) {
        Frame* f2 = neighbor[i];
        matcher.searchForFuse(f2, 1.5);
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
        m.searchForFuse(keyFrame, 1.5);
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
    cout << "[localmap] start culling ... " << endl;
    list<MapPoint*>::iterator itr = newMapPoints.begin();
    int count = 0;
    while (itr != newMapPoints.end()) {
        MapPoint* p = *itr;
        /*
        if (p == NULL) {
            cout << "[local map] null point at : " << count << endl;
            count ++;
            itr = newMapPoints.erase(itr);
            continue;
        }*/
        if (!p->good) {
            itr = newMapPoints.erase(itr);
            continue;
        }
        if (keyFrame->keyFrameId - p->observation.firstKFId() >= 2 && p->observation.observations.size() <= 2) {
            count ++;
            p->destroy();
            itr = newMapPoints.erase(itr);
            continue;
        }
        if (keyFrame->keyFrameId - p->observation.firstKFId() >= 1 && p->observation.observations.size() < 2 && p->observation.firstKFId() ==0) {
            count ++;
            p->destroy();
            itr = newMapPoints.erase(itr);
        }
        if (keyFrame->keyFrameId - p->observation.firstKFId() >= 3) {
            count ++;
            itr = newMapPoints.erase(itr);
            continue;
        }
        itr ++;
    }
    cout << "[localmap] finish culling ... " << endl;
    cout << "[localmap] culling num: " << count << endl;
}

void LocalMap::localOptimize(Frame* keyFrame) {
    list<Frame*> localFrames;
    list<MapPoint*> localPoints;
    list<Frame*> fixedFrames;
    set<int> mark;
    set<int> pointMark;
    set<int> ffMark;
    mark.clear();
    pointMark.clear();
    ffMark.clear();
    localFrames.clear();
    fixedFrames.clear();
    localFrames.push_back(keyFrame);
    mark.insert(keyFrame->keyFrameId);
    //fixedFrames.push_back();
    mapInfo.getNeighborKF(keyFrame, neighbor, 15, 5);
    for (int i = 0; i < neighbor.size(); i++) {
        Frame* f = neighbor[i];
        //if (f->keyFrameId != 0) {
            localFrames.push_back(f);
            mark.insert(f->keyFrameId);
        //} else {
        //    fixedFrames.push_back(f);
        //}
    }

    localPoints.clear();
    list<Frame*>::iterator itr= localFrames.begin();
    while (itr != localFrames.end()) {
        Frame* f = *itr;
        for (int i = 0; i < f->features->keyPointsNum; i++) {
            Feature& fe = f->features->keyPoints[i];
            if (!fe.mapPoint || !fe.mapPoint->good) {
                continue;
            }
            if (pointMark.count(fe.mapPoint->pointId) > 0) {
                continue;
            }
            pointMark.insert(fe.mapPoint->pointId);
            localPoints.push_back(fe.mapPoint);
        }
        itr ++;
    }

    list<MapPoint*>::iterator pitr = localPoints.begin();
    while (pitr != localPoints.end()) {
        MapPoint* p = *pitr;
        map<Frame*, int>& ob = p->observation.observations;
        map<Frame*, int>::iterator fitr = ob.begin();
        while (fitr != ob.end()) {
            Frame* f = fitr->first;
            if (mark.count(f->keyFrameId) == 0 && ffMark.count(f->keyFrameId) == 0) {
                fixedFrames.push_back(f);
                ffMark.insert(f->keyFrameId);
            }
            fitr ++;
        }
        pitr ++;
    }
    // local bundle adjustment
    Optimizer::localBA(localFrames, localPoints, fixedFrames);
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
    fuseMapPoints(keyFrame);
    cout << "[local map] optimize [frameid]: " << keyFrame->frameId << " start optimize " << endl;
    localOptimize(keyFrame);
    cout << "[local map] optimize [frameid]: " << keyFrame->frameId << " finish optimize " << endl;
    //keyFrames.push_back (keyFrame);
}

LocalMap::LocalMap() {
    clear();
}

void LocalMap::initialize(Frame* keyFrame) {
    newMapPoints.clear();
    for (int i = 0; i < keyFrame->features->keyPointsNum; i++) {
        Feature& f = keyFrame->features->keyPoints[i];
        if (f.mapPoint == NULL) {
            continue;
        }
        newMapPoints.push_back(f.mapPoint);
    }
}

void LocalMap::clear() {
    //keyFrames.clear();
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
