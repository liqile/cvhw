#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "Frame.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

namespace lqlslam {

    class Tracking {
        private:

        /*
         * last frame during tracking
         */
        Frame* lastFrame;

        /*
         * setLastFrame
         * @param Frame* frame
         * function: set frame as lastFrame
         */
        void setLastFrame(Frame* frame);

        /*
         * filterOutlier
         * @param Frame* frame, current tracking frame
         * function: after tracking, some matches in frame will
         *     be regared as outlier. just filter them
         */
        void filterOutlier(Frame* frame);

        public:
        /*
         * Tracking
         * @param
         * function: construction
         */
        Tracking();

        /*
         * initialize
         * @param const Frame* frame, first camera frame
         * function: set the first frame to initialize tracker
         *         the pose of first frame is set to I(4, 4)
         */
        void initialize(Frame* frame);

        /*
         * trackLastFrame
         * @param Frame* frame, current Frame
         * function:
         *         track current frame from last frame
         *         and set pose of the frame in world system in frame
         *         return number of map points tracked
         */
        int trackLastFrame(Frame* frame);

        /*
         * trackLocalMap
         * @param Frame* frame, current frame
         * function: track current frame from local map
         */
        int trackLocalMap(Frame* frame);

        /*
         * track
         * @param Frame* frame, current frame
         * function: track current frame
         */
        int track(Frame* frame);
    };

}

#endif
