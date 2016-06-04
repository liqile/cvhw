#include "Initializer.h"
#include "Svd.h"
#include "MatcherDrawer.h"

namespace lqlslam {

bool Initializer::initialize() {
    Frame* frame1 = frames[0];
    Frame* frame2 = frames[1];
    ORBmatcher matcher(frame1);
    vector<cv::DMatch> matches;
    matcher.searchForInit (frame2, matches);
    MatcherDrawer::drawInitMatch (frame1, frame2, matches);

    vector<cv::Point2f> p1;
    vector<cv::Point2f> p2;
    p1.clear();
    p2.clear();
    for (int i = 0; i < matches.size(); i++) {
        const cv::DMatch& match = matches[i];
        int i1 = match.queryIdx;
        int i2 = match.trainIdx;
        cv::Point2f pt1 = frame1->features->rawKeyPoints[i1].pt;
        cv::Point2f pt2 = frame2->features->rawKeyPoints[i2].pt;
        p1.push_back(pt1);
        p2.push_back(pt2);
    }

    vector<bool> mask;
    cv::Mat fundD = Svd::getFundamental(p1, p2, mask);
    cv::Mat fund;
    fundD.convertTo (fund, CV_32F);

    vector<cv::DMatch> matchesE;
    matchesE.clear();
    for (int i = 0; i < matches.size(); i++) {
        if (mask[i]) {
            matchesE.push_back(matches[i]);
        }
    }

    cout << "fund: " << fund << endl;
    MatcherDrawer::drawInitMatch (frame1, frame2, matchesE);

    vector<cv::KeyPoint> kp1;
    vector<cv::KeyPoint> kp2;
    kp1.clear();
    kp2.clear();
    for (int i = 0; i < matchesE.size(); i++) {
        const cv::DMatch& match = matchesE[i];
        int i1 = match.queryIdx;
        int i2 = match.trainIdx;
        cv::KeyPoint kpt1 = frame1->features->keyPoints[i1].keyPoint;
        cv::KeyPoint kpt2 = frame2->features->keyPoints[i2].keyPoint;
        kp1.push_back(kpt1);
        kp2.push_back(kpt2);
    }

    vector<cv::Mat> points;

    Pose pose;
    bool ok = Svd::getRt(kp1, kp2, fund, pose);
    if (!ok) {
        cout << "Initialize failed" << endl;
        return false;
    }

    frame2->setPose(pose.mTcw);
    const Pose& pose1 = frame1->pose;
    const Pose& pose2 = frame2->pose;
    cout << "pose1: " << pose1.mTcw << endl;
    cout << "pose2: " << pose2.mTcw << endl;
    frame1->becomeKeyframe();
    int count = 0;
    for (int i = 0; i < matchesE.size(); i++) {
        const cv::DMatch& match = matchesE[i];
        int i1 = match.queryIdx;
        int i2 = match.trainIdx;
        cv::KeyPoint kpt1 = frame1->features->keyPoints[i1].keyPoint;
        cv::KeyPoint kpt2 = frame2->features->keyPoints[i2].keyPoint;
        cv::Mat pos = Pose::triangular(pose1, pose2, kpt1, kpt2);
        if (pos.empty()) {
            continue;
        }
        count ++;
        MapPoint* point = new MapPoint(pos, frame1, i1);
        frame2->features->setMapPoint (i2, point);
    }
    cout << "[init] number of mappoints " << count << endl;
    localMap->initialize(frame1);
    frame2->becomeKeyframe();
    localMap->addKeyFrame(frame2);
    //frame2->setPose ();

    return true;
}

Initializer::Initializer(Tracking* tracker, LocalMap* localMap) {
    frames.clear();
    lastFrame = NULL;
    this->tracker = tracker;
    this->localMap = localMap;
    need = true;
}

bool Initializer::needInitialize() {
    return need;
}

void Initializer::addFrame(Frame* frame) {
#if SEMI_MONOCULAR
    frames.push_back(frame);
    tracker->initialize(frame);
    frame->generate3dPoints();
    lastFrame = frame;
    frame->becomeKeyframe();
    localMap->initialize(lastFrame);
    need = false;
    return;
#endif
    frames.push_back(frame);
    tracker->initialize(frame);
    lastFrame = frame;
    if (frames.size() >= 2) {
        need = !initialize();
    }
}

Frame* Initializer::getLastFrame() {
    return lastFrame;
}

}
