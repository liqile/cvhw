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

void MatcherCounter::addLastProjectNum() {
    lastProjectNum ++;
}

MatcherCounter trackingMatcherCounter(0);
MatcherCounter mappingMatcherCounter(1);

}
