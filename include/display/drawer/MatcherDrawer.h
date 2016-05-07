#ifndef MATCHER_DRAWER_H
#define MATCHER_DRAWER_H

#include "Frame.h"
#include "Features.h"
#include "display.h"

namespace lqlslam {

class MatcherDrawer {
private:
    /*
     * 0 for tracking, 1 for mapping
     */
    int tag;
    /*
     * img of matches
     */
    cv::Mat match;
public:
    /*
     * MatcherDrawer
     * @param int tag, 0 for tracking, 1 for mapping
     * function: constructor
     */
    MatcherDrawer(int tag);
    /*
     * drawMatches
     * @param const Frame* curr, current frame (first frame)
     * @param const Frame* last, last frame (second frame)
     * @param const vector<int>& lastIdx, <i, lastIdx[i]> is a pair of matches of first frame and second frame
     * function: draw matches of current frame and last frame in two images
     */
    void drawMatches(const Frame* curr, const Frame* last, const vector<int>& lastIdx);
    /*
     * drawMatchOneByOne
     * @param const Frame* curr, current frame (first frame)
     * @param const Frame* last, last frame (second frame)
     * @param const cv::Mat& F12, fundamental matrix of frame1 and frame2,
     *         empty for no such fundamental matrix
     * @param const vector<int>& lastIdx, <i, lastIdx[i]> is a pair of matches of first frame and second frame
     * function: draw matches one by one
     */
    void drawMatchOneByOne(const Frame* curr, const Frame* last, const cv::Mat& F12, const vector<int>& lastIdx);
    /*
     * drawTrackingMatches
     * @param const Frame* curr, current frame (first frame)
     * @param const Frame* last, last frame (second frame)
     * @param const vector<int>& lastIdx, <i, lastIdx[i]> is a pair of matches of first frame and second frame
     * function: draw matches in a single image
     */
    void drawTrackingMatches(const Frame* curr, const Frame* last, const vector<int>& lastIdx);
};

}

#endif
