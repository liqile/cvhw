#ifndef DRAWERS_H
#define DRAWERS_H

#include "TrackingDrawer.h"
#include "MatcherCounter.h"

namespace lqlslam {
    extern MatcherCounter trackingMatcherCounter;
    extern MatcherCounter mappingMatcherCounter;
    extern TrackingDepthDrawer trackingDepthDrawer;
}

#endif
