#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <vector>

#include "Frame.h"
#include "CovGraph.h"

using namespace std;

namespace lqlslam {

class Map {
    private:
    /*
     * keyframes
     */
    vector<Frame*> keyFrames;
    /*
     * covisbility graph
     */
    CovGraph covGraph;
    public:
    /*
     * Map
     * function: constructor
     */
    Map();
    /*
     * getNeighborKf
     * @param Frame* frame, current query frame
     * @param vector<Frame*>& neigh, store neighbor keyframes
     *     of current frame in map when return
     * @param int plb, lowerbound of common points.
     * @param int lim, limit of frames return, -1 for all
     * function: get neighbor keyframes of current frame in map by search covisibility graph
     *     result stored in neigh, sorted by common number of mappoints, in desc order
     */
    void getNeighbors(Frame* frame, vector<Frame*>& neigh, int plb, int lim);
    /*
     * updateKeyFrame
     * @param Frame* keyFrame, keyframe to update
     * function: update keyFrame in map
     *    - update covisibility graph
     */
    void updateKeyFrame(Frame* frame);
    /*
     * addKeyFrame
     * @param Frame* keyFrame,
     * function: add keyFrame in map
     *    - update covisibility graph
     */
    void addKeyFrame(Frame* frame);
    /*
     * delKeyFrame
     * @param Frame* keyFrame,
     * function: delete keyFrame in map
     */
    void delKeyFrame(Frame* frame);
};

/*
 * map
 */
extern Map mapInfo;

}

#endif
