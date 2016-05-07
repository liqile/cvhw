#ifndef DRAWERS_H
#define DRAWERS_H

#include "TrackingDrawer.h"
#include "MatcherDrawer.h"

namespace lqlslam {

    extern MatcherDrawer trackingMatcherDrawer;
    extern MatcherDrawer mappingMatcherDrawer;
    extern TrackingDepthDrawer trackingDepthDrawer;

}

#endif
