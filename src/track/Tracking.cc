#include "Tracking.h"
#include "Map.h"
#include "drawers.h"
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
            trackingMatcherCounter.reduceMatch(i);
        }
    }
}

void Tracking::setMapPoint(Frame* f1, Frame* f2) {
    for (int i = 0; i < f1->features->keyPointsNum; i++) {
        int idx2 = trackingMatcherCounter.lastIdx[i];
        if (idx2 == -1) {
            continue;
        }
        Feature& fe1 = f1->features->keyPoints[i];
        Feature& fe2 = f2->features->keyPoints[idx2];
        fe1.mapPoint = fe2.mapPoint;
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
    cout << "[tracking]last frame id: " << lastFrame->frameId << endl;
    frame->setPose(lastFrame->pose.mTcw);
    ORBmatcher matcher(frame);
    matcher.searchLastFrame(lastFrame, 14);
    setMapPoint(frame, lastFrame);
    int good = Optimizer::poseOptimize(frame);

    filterOutlier(frame);
    trackingMatcherDrawer.drawMatches(frame, lastFrame, trackingMatcherCounter.lastIdx);
    trackingMatcherCounter.print();
    displayer->show();
    return good;
}

int Tracking::trackLocalMap(Frame* frame) {
    neigh.clear();
    mapInfo.getNeighbors(frame, neigh, -1, -1);
    ORBmatcher matcher(frame);
    for (int i = 0; i < neigh.size(); i++) {
        Frame* ref = neigh[i];
        matcher.searchKeyFrame(ref, 2);
        trackingMatcherDrawer.drawMatches(frame, ref, trackingMatcherCounter.lastIdx);
        trackingMatcherCounter.print();
        setMapPoint(frame, ref);
        cout << "[tracking]match local map of : " << ref->frameId << endl;
        displayer->show();
    }
    int good = Optimizer::poseOptimize(frame);
    cout << "[tracking]track local map: " << frame->frameId << " with " << good << " map point" << endl;
    cout << "[tracking]before filter outlier" << endl;
    filterOutlier(frame);
    cout << "[tracking]after filter outlier" << endl;
    //displayer->show();
    //cout << "track local map after show matches"
    return good;
}

int Tracking::track(Frame* frame) {
    int good = trackLastFrame(frame);
    cout << "track last frame: " << frame->frameId << " with " << good << " map points" << endl;
    displayer->show();
    //filterOutlier(frame);
    good = trackLocalMap(frame);
    //cout << "before filter outlier" << endl;
    //filterOutlier(frame);
    //cout << "after filter outlier" << endl;
    setLastFrame(frame);
    cout << "[tracking]after set last frame" << endl;
    return good;
}

}
