#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "Frame.h"

namespace lqlslam {

class Frame;

class Map {
    public:
    Map();
    void clear();
    void addKeyFrame(Frame* keyFrame);
    std::vector<Frame*> keyFrames;
};

}

#endif
