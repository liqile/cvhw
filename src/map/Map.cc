#include "Map.h"

namespace lqlslam {

Map mapInfo;

Map::Map() {
    keyFrames.clear();
}

void Map::getNeighbors(Frame *frame, vector<Frame *> &neigh, int plb, int lim) {
    neigh.clear();
    covGraph.getNeighbors(frame, neigh, lim, plb);
    cout << "[local map] after get neighbor in cov graph, frameid: " << frame->frameId << endl;
}

void Map::updateKeyFrame(Frame *frame) {
    covGraph.updateKeyFrame(frame);
}

void Map::addKeyFrame(Frame *frame) {
    cout << "[local map] in map, try to add key frame, frameid: " << frame->frameId << endl;
    for (int i = 0; i < frame->features->keyPointsNum; i++) {
        Feature& f = frame->features->keyPoints[i];
        if (f.mapPoint == NULL || !f.mapPoint->good) {
            continue;
        }
        f.mapPoint->addObservation(frame, i);
        f.mapPoint->observation.updateDescriptor();
    }
    covGraph.addKeyFrame(frame);
    cout << "[local map] in map, after adding key frame, frameid: " << frame->frameId << endl;
}

void Map::delKeyFrame(Frame* frame) {
    cout << "[local map] in map, try to del key frame, frameid: " << frame->frameId << endl;
    frame->badKeyFrame = true;
    covGraph.delKeyFrame(frame);
    for (int i = 0; i < frame->features->keyPointsNum; i++) {
        Feature& f = frame->features->keyPoints[i];
        if (f.mapPoint == NULL || !f.mapPoint->good) {
            continue;
        }
        f.mapPoint->decObservation(frame);
    }
    cout << "[local map] in map, after del key frame, frameid: " << frame->frameId << endl;
}

}
