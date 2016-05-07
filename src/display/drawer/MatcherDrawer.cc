#include "MatcherDrawer.h"

namespace lqlslam {

MatcherDrawer::MatcherDrawer(int tag) {
    this->tag = tag;
}

void MatcherDrawer::drawMatches(const Frame *curr, const Frame *last, const vector<int>& lastIdx) {
    //print();
    const vector<cv::KeyPoint>& kp2 = last->features->rawKeyPoints;
    const vector<cv::KeyPoint>& kp1 = curr->features->rawKeyPoints;
    const cv::Mat& img2 = last->rawData->img;
    const cv::Mat& img1 = curr->rawData->img;
    vector<cv::DMatch> matches;
    matches.clear();
    for (int i = 0; i < lastIdx.size(); i++) {
        if (lastIdx[i] == -1) {
            continue;
        }
        cv::DMatch match;
        match.queryIdx = lastIdx[i];
        match.trainIdx = i;
        matches.push_back(match);
    }
    cv::drawMatches(img2, kp2, img1, kp1, matches, match, cv::Scalar(255, 0, 0));
    if (tag == 1) {
        displayer->setMatchGraph(match);
    } else {
        displayer->setTrackingMatchGraph(match);
    }
}


void MatcherDrawer::drawMatchOneByOne(const Frame *curr, const Frame *last, const cv::Mat &F12, const vector<int>& lastIdx) {
    const vector<cv::KeyPoint>& kp2 = last->features->rawKeyPoints;
    const vector<cv::KeyPoint>& kp1 = curr->features->rawKeyPoints;
    const cv::Mat& img2 = last->rawData->img;
    const cv::Mat& img1 = curr->rawData->img;
    for (int i = 0; i < lastIdx.size(); i++) {
        if (lastIdx[i] == -1) {
            continue;
        }
        vector<cv::DMatch> matches;
        matches.clear();
        cv::DMatch m;
        m.queryIdx = lastIdx[i];
        m.trainIdx = i;
        matches.push_back(m);
        cv::drawMatches(img2, kp2, img1, kp1, matches, match, cv::Scalar(255, 0, 0));
        float a, b, c;
        Pose::getEpipolarLine (kp1[i], F12, a, b, c);
        if (b != 0) {
            //y = - a / b * x - c / b;
            float x0 = 0;
            float y0 = - a / b * x0 - c / b;
            float x1 = img2.cols - 1;
            float y1 = - a / b * x1 - c / b;
            cv::Point2f p0(x0, y0);
            cv::Point2f p1(x1, y1);
            cv::line(match, p0, p1, cv::Scalar(0, 0, 255));
        } else {
            //x = - b / a * y - c / a;
            float y0 = 0;
            float x0 = - b / a * x0 - c / a;
            float y1 = img2.rows - 1;
            float x1 = - b / a * x1 - c / a;
            cv::Point2f p0(x0, y0);
            cv::Point2f p1(x1, y1);
            cv::line(match, p0, p1, cv::Scalar(0, 0, 255));
        }
        if (tag == 1) {
            displayer->setMatchGraph(match);
        } else {
            displayer->setTrackingMatchGraph(match);
        }
        if (!displayer->show()) {
            break;
        }
    }
}

void MatcherDrawer::drawTrackingMatches(const Frame *curr, const Frame *last, const vector<int>& lastIdx) {
    const vector<cv::KeyPoint>& kp1 = curr->features->rawKeyPoints;
    const vector<cv::KeyPoint>& kp2 = last->features->rawKeyPoints;
    cv::Mat img1 = curr->rawData->img.clone();
    for (int i = 0; i < lastIdx.size(); i++) {
        if (lastIdx[i] == -1) {
            continue;
        }
        const cv::KeyPoint& k1 = kp1[i];
        const cv::KeyPoint& k2 = kp2[lastIdx[i]];
        cv::line(img1, k1.pt, k2.pt, cv::Scalar(255, 0, 0));
    }
    displayer->setTrackingMatchGraph(img1);
}

}
