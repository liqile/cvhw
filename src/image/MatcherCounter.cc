#include "MatcherCounter.h"

namespace lqlslam {

MatcherCounter::MatcherCounter(int tag) {
    this->tag = tag;
}

void MatcherCounter::reset(const Features* curr, const Features* last) {
    lastIdx.clear();
    for (int i = 0; i < curr->keyPointsNum; i++) {
        lastIdx.push_back(-1);
    }
    currIdx.clear();
    for (int i = 0; i < last->keyPointsNum; i++) {
        currIdx.push_back(-1);
    }
    lastKeyPointNum = last->keyPointsNum;
    lastMapPointNum = 0;
    for (int i = 0; i < last->keyPointsNum; i++) {
        const Feature& feature = last->keyPoints[i];
        if (feature.mapPoint == NULL) {
            continue;
        }
        lastMapPointNum ++;
    }
    lastProjectNum = 0;
    lastMatchNum = 0;
    lastMatchFilterNum = 0;
}

void MatcherCounter::print() {
    cout << "[Matcher] last frame keypoints num: " << lastKeyPointNum << endl;
    cout << "[Matcher] last frame map point num: " << lastMapPointNum << endl;
    cout << "[Matcher] last frame map point reproject num: " << lastProjectNum << endl;
    cout << "[Matcher] last frame matches num: " << lastMatchNum << endl;
    cout << "[Matcher] last frame matches num (after filter): " << lastMatchFilterNum << endl;
}

void MatcherCounter::addMatch(int currIdx, int lastIdx) {
    this->lastIdx[currIdx] = lastIdx;
    this->currIdx[lastIdx] = currIdx;
    lastMatchNum ++;
    lastMatchFilterNum ++;
}

void MatcherCounter::reduceMatch(int currIdx) {
    lastIdx[currIdx] = -1;
    lastMatchFilterNum --;
}

void MatcherCounter::drawMatches(const Frame *curr, const Frame *last) {
    print();
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

void MatcherCounter::drawMatchOneByOne(const Frame *curr, const Frame *last, const cv::Mat &F12) {
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

void MatcherCounter::drawTrackingMatches(const Frame *curr, const Frame *last) {
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

void MatcherCounter::addLastProjectNum() {
    lastProjectNum ++;
}

}
