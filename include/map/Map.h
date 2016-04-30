#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <vector>

#include "Frame.h"

using namespace std;

namespace lqlslam {

class Map {
    private:
    /*
     * keyframes
     */
    vector<Frame*> keyFrames;
    public:
    Map();
    /*
     * getNeighborKf
     * @param const Frame* frame, current query frame
     * @param vector<Frame*>& neigh, store neighbor keyframes
     *     of current frame in map when return
     * function: get neighbor keyframes of current frame in map
     *     neigh[0] has maximum number of mappoints in frame
     */
    void getNeighborKF(const Frame* frame, vector<Frame*>& neigh);
};

extern Map mapInfo;

}

#endif
