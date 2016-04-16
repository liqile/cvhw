#include "LocalMap.h"
#include "Pose.h"
#include "ORBmatcher.h"
#include "drawers.h"
#include "display.h"

namespace lqlslam {

void LocalMap::triangularPoints(Frame* f1, Frame* f2) {
    const vector<int>& matchOfF1 = mappingMatcherCounter.lastIdx;
    const vector<int>& matchOfF2 = mappingMatcherCounter.currIdx;
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
    cout << "tot triangular matches: " << tot << endl;
    cout << "tot map point: " << num << endl;
}

void LocalMap::createMapPoints(Frame *keyFrame) {
    vector<Frame*> neighbor;
    getNeighborKeyFrames(keyFrame, neighbor);
    ORBmatcher matcher(keyFrame);
    for (int i = 0; i < neighbor.size(); i++) {
        Frame* neigh = neighbor[i];
        cout << "triangular : " << keyFrame->frameId << " with " << neigh->frameId << endl;
        matcher.searchByTriangular(neigh);
        triangularPoints(keyFrame, neigh);
        displayer->show();
    }
}

void LocalMap::processNewKeyFrame(Frame *keyFrame) {
    createMapPoints(keyFrame);
    keyFrames.push_back (keyFrame);
}

LocalMap::LocalMap() {
    clear();
}

void LocalMap::clear() {
    keyFrames.clear();
}

void LocalMap::addKeyFrame(Frame* keyFrame) {
    processNewKeyFrame(keyFrame);
}

void LocalMap::getNeighborKeyFrames(const Frame* frame, std::vector<Frame*>& frames) {
    frames.clear();
    for (int i = 0; i < keyFrames.size(); i++) {
        Frame* f = keyFrames[i];
        //if (frame->keyFrameId - f->keyFrameId <= 3) {
        if (f->frameId == 0) {
            frames.push_back(f);
        }
    }
}

}
