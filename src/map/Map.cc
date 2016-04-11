#include "Map.h"
#include "Pose.h"

namespace lqlslam {

Map::Map() {
    clear();
}

void Map::clear() {
    keyFrames.clear();
}

void Map::addKeyFrame(Frame* keyFrame) {
    keyFrames.push_back (keyFrame);
}

}
