#include "Map.h"

namespace lqlslam {

Map mapInfo;

void Map::getNeighborKF(const Frame *frame, vector<Frame *> &neigh) {
    neigh.clear();
    const Features* features = frame->features;
    map<Frame*, int> counter;
    counter.clear();
    for (int i = 0; i < features->keyPointsNum; i++) {
        const Feature& feature = features->keyPoints[i];
        const MapPoint* p = feature.mapPoint;
        if (p == NULL) {
            continue;
        }
        const map<Frame*, int>& obs = p->observation.observations;
        map<Frame*, int>::const_iterator it = obs.begin();
        while (it != obs.end()) {
            counter[it->first] ++;
            it ++;
        }
    }
    map<Frame*, int>::iterator it = counter.begin();
    int max = 0;
    while (it != counter.end()) {
        neigh.push_back(it->first);
        int idx = neigh.size() - 1;
        if (it->second > max) {
            max = it->second;
            Frame* tmp = neigh[0];
            neigh[0] = neigh[idx];
            neigh[idx] = tmp;
        }
        it ++;
    }
}

}
