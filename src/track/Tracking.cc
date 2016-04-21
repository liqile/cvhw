#include "Tracking.h"
#include "Map.h"

namespace lqlslam {

void Tracking::setLastFrame(Frame *frame) {
    lastFrame = frame;
#if ONLY_TRACKING
    lastFrame->generate3dPoints();
#endif
}

void Tracking::filterOutlier(Frame* frame) {
    for (int i = 0; i < frame->features->keyPointsNum; i++) {
        Feature& feature = frame->features->keyPoints[i];
        if (feature.mapPoint == NULL) {
            continue;
        }
        if (feature.isOutlier) {
            feature.mapPoint->trackLog.setTrackMatchedFrame(-1);
            feature.mapPoint = NULL;
        }
    }
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
    return good;
}

int Tracking::trackLocalMap(Frame* frame) {
    vector<Frame*> neigh;
    mapInfo.getNeighborKF(frame, neigh);
    ORBmatcher matcher(frame);
    for (int i = 0; i < neigh.size(); i++) {
        Frame* ref = neigh[i];
        matcher.searchByProject (ref, 2);
        cout << "match local map of : " << ref->frameId << endl;
        displayer->show();
    }
    int good = Optimizer::poseOptimize(frame);
    cout << "track local map: " << frame->frameId << " with " << good << " map point" << endl;
    //displayer->show();
    //cout << "track local map after show matches"
    return good;
}

int Tracking::track(Frame* frame) {
    int good = trackLastFrame(frame);
    cout << "track last frame: " << frame->frameId << " with " << good << " map points" << endl;
    displayer->show();
    filterOutlier(frame);
    good = trackLocalMap(frame);
    cout << "before filter outlier" << endl;
    filterOutlier(frame);
    cout << "after filter outlier" << endl;
    setLastFrame(frame);
    cout << "after set last frame" << endl;
    return good;
}

}
