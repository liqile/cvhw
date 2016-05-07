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
     * @param int plb, lowerbound of common points. default as -1
     * @param int lim, limit of frames return, defualt as -1 for all
     * function: get neighbor keyframes of current frame in map
     *     neigh[0] has maximum number of mappoints in frame
     */
    void getNeighborKF(const Frame* frame, vector<Frame*>& neigh, int plb = -1, int lim = -1);
};

extern Map mapInfo;

}

#endif
