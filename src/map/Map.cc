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
    covGraph.addKeyFrame(frame);
    cout << "[local map] in map, after adding key frame, frameid: " << frame->frameId << endl;
}

}
