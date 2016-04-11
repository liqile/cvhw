#include "Tracking.h"

namespace lqlslam {

void Tracking::setLastFrame(Frame *frame) {
    lastFrame = frame;
#if ONLY_TRACKING
    lastFrame->becomeKeyframe();
#endif
}

Tracking::Tracking() {
    lastFrame = NULL;
}

void Tracking::initialize(Frame *frame) {
    frame->setPose(cv::Mat::eye(4, 4, CV_32F));
    setLastFrame(frame);
}

int Tracking::trackLastFrame(Frame* frame) {
    cout << "last frame id: " << lastFrame->frameId << endl;
    frame->setPose(lastFrame->pose.mTcw);
    ORBmatcher matcher(frame);
    matcher.searchByProject(lastFrame, 14);
    int good = Optimizer::poseOptimize(frame);
    //int good = frame->features->keyPointsNum;
    setLastFrame(frame);
    return good;
}

}
