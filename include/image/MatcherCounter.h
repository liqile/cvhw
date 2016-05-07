#ifndef MATCHER_COUNTER_H
#define MATCHER_COUNTER_H

#include "Frame.h"
#include "Features.h"
#include "display.h"

namespace lqlslam {

class MatcherCounter {
public:
    /*
     * number of keypoints in last frame
     */
    int lastKeyPointNum;
    /*
     * number of mappoints in last frame
     */
    int lastMapPointNum;
    /*
     * number of map points successfully project from last frame to current frame
     */
    int lastProjectNum;
    /*
     * all matches num before filter
     */
    int lastMatchNum;
    /*
     * matches num after filter
     */
    int lastMatchFilterNum;
    /*
     * match index with trackref frame (last frame also called second frame)
     * -1 for no such match
     */
    vector <int> lastIdx;
    /*
     * match index with current frame (also called first frame)
     * -1 for no such match
     */
    vector <int> currIdx;
    /*
     * img of match
     */
    cv::Mat match;
    /*
     * 0 for tracking, 1 for mapping
     */
    int tag;
public:
    /*
     * MatcherCounter
     * @param int tag, 0 for tracking, 1 for mapping
     * function: constructor
     */
    MatcherCounter(int tag);
    /*
     * reset
     * @param const Features* curr, features of curr frame
     * @param const Features* last, features of last frame
     * function: reset counter
     */
    void reset(const Features* curr, const Features* last);
    /*
     * print
     * function: print counter information
     */
    void print();
    /*
     * addMatch
     * @param int currIdx, feature idx of current frame
     * @param int lastIdx, feature idx of last frame
     * function: log information of a match
     */
    void addMatch(int currIdx, int lastIdx);
    /*
     * reduceMatch
     * @param int currIdx, feature idx of current frame
     * function: unlog information of a match
     */
    void reduceMatch(int currIdx);
    /*
     * addLastProjectNum
     * function: increase lastProjectNum by 1
     */
    void addLastProjectNum();
};

extern MatcherCounter trackingMatcherCounter;
extern MatcherCounter mappingMatcherCounter;

}

#endif
