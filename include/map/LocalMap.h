#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "Frame.h"

namespace lqlslam {

class Frame;

class LocalMap {
    private:

    //void addPoint(Frame* f1, Frame* f2, int i1, int i2);
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
    void clear();
    /*
     * addKeyFrame(Frame* keyFrame)
     * @param Frame* keyFrame, a new key frame
     * function: a interface for add a new keyframe in other thread (tracking thread)
     */
    void addKeyFrame(Frame* keyFrame);
    std::vector<Frame*> keyFrames;
    /*
     * getNeighborKeyFrames
     * @param const Frame* frame, a key frame
     * @param std::vector<Frame*> frames, store neighbor keyframes of
     *         frame when return
     * function: get neighbor keyframes of frame
     */
    void getNeighborKeyFrames(const Frame* frame, std::vector<Frame*>& frames);
};

}

#endif
