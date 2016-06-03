#include "Initializer.h"

namespace lqlslam {

Initializer::Initializer(Tracking* tracker, LocalMap* localMap) {
    frames.clear();
    lastFrame = NULL;
    this->tracker = tracker;
    this->localMap = localMap;
}

bool Initializer::needInitialize() {
    if (frames.size() < 1) {
        return true;
    }
    return false;
}

void Initializer::addFrame(Frame* frame) {
    frames.push_back(frame);
    tracker->initialize(frame);
    frame->generate3dPoints();
    lastFrame = frame;
    frame->becomeKeyframe();
    localMap->initialize(lastFrame);
}

Frame* Initializer::getLastFrame() {
    return lastFrame;
}

}
