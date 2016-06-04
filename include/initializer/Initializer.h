#ifndef INITIALIZER_H
#define INITIALIZER_H

#include "Frame.h"
#include "Map.h"
#include "Tracking.h"
#include <vector>

using namespace std;

namespace lqlslam {

class Initializer {
private:
    vector <Frame*> frames;
    Tracking* tracker;
    LocalMap* localMap;
    Frame* lastFrame;
    bool initialize();
    bool need;
public:
    Initializer(Tracking* tracker, LocalMap* localMap);
    bool needInitialize();
    void addFrame(Frame* frame);
    Frame* getLastFrame();
};

}

#endif
