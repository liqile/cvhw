#ifndef LOCAL_MAP_H
#define LOCAL_MAP_H

#include "MapPoint.h"
#include "Frame.h"

namespace lqlslam {

class Frame;
class MapPoint;

class LocalMap {
    private:
    /*
     * neighbor keyframe of current keyframe
     */
    //vector<Frame*> neighbor;


    /*
     * newly created map points
     */
    std::list<MapPoint*> newMapPoints;
    /*
     * triangularPoints
     * @param Frame* f1, first frame in matcher
     * @param Frame* f2, second frame in matcher
     * function: generate 3d map points of f1 and f2 by
     *         matches and triangulation
     */
    void triangularPoints(Frame* f1, Frame* f2);
    /*
     * createMapPoints
     * @param Frame* keyFrame, a new key frame
     * function: generate 3d map points in unmatched keypoints of new key frame
     *         by using triangulation method with neighbor keyframes
     */
    void createMapPoints(Frame* keyFrame);
    /*
     * fuseMatches
     * @param Frame* f1, first frame
     * @param Frame* f2, second frame
     * function: fuse mappoints according to matches
     */
    void fuseMatches(Frame* f1, Frame* f2);
    /*
     * fuseMapPoint
     * @param Frame* keyFrame
     * function: fuse mappoint in keyframe and its neighbor keyframes
     */
    void fuseMapPoints(Frame* keyFrame);
    /*
     * processTrackedPoints
     * @param Frame* keyFrame
     * function: process tracked map points
     */
    void processTrackedPoints(Frame* keyFrame);
    /*
     * cullMapPoint
     * @param Frame* keyFrame, current key frame
     * function: cull map points in newMapPoints list
     */
    void cullMapPoint(Frame* keyFrame);
    /*
     * needCullKF
     * @param Frame* keyFrame, a key frame
     * function: check if need to the keyframe
     *     condition: 90% mappoints in a keyframe was observed in other
     *     key frames at better octave
     */
    bool needCullKF(Frame* keyFrame);
    /*
     * cullKeyFrame
     * @param Frame* keyFrame, current key frame
     * function: cull the key frame
     */
    void cullKeyFrame(Frame* keyFrame);
    /*
     * localOptimize
     * @param Frame* keyFrame, current key frame
     * function: optimize frame poses and mappoints in local map
     *     of current key frame (local bundle adjustment)
     */
    void localOptimize(Frame* keyFrame);
    /*
     * processNewKeyFrame
     * @param Frame* keyFrame, a new key frame
     * function: deal with a keyframe received by local map thread
     */
    void processNewKeyFrame(Frame* keyFrame);
    public:
    /*
     * LocalMap
     * function: constructor
     */
    LocalMap();
    /*
     * initialize
     * @param Frame* keyFrame, first key frame
     * function: initialize map
     */
    void initialize(Frame* keyFrame);
    /*
     * clear
     */
    void clear();
    /*
     * addKeyFrame(Frame* keyFrame)
     * @param Frame* keyFrame, a new key frame
     * function: a interface for add a new keyframe in other thread (tracking thread)
     */
    void addKeyFrame(Frame* keyFrame);
    /*
     * keyframes
     */
    std::vector<Frame*> keyFrames;
};

}

#endif
