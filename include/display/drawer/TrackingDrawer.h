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
public:
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
     * drawMatches
     * @param const Frame* curr, current frame
     * @param const Frame* last, last frame
     * function: draw matches of current frame and last frame
     */
    void drawMatches(const Frame* curr, const Frame* last);
    /*
     * addLastProjectNum
     * function: increase lastProjectNum by 1
     */
    void addLastProjectNum();
};

/*
 * TrackingDepthDrawer
 * draw depth information of pixels after each tracking
 * only used in "ONLY_TRACKING" mode, otherwise,
 *     we can not easily get depth of each pixel
 */
class TrackingDepthDrawer {
    private:
    /*
     * point cloud of tracked frames
     */
    PointCloud::Ptr* trackedPcl;
    public:
    /*
     * TrackingDepthDrawer
     * constructor
     */
    TrackingDepthDrawer();
    /*
     * singleFrameDepth
     * const cv::Mat& rgb, rgb Mat of current frame
     * const cv::Mat& depth, depth Mat of current frame (channel 1, 32-bit float)
     * function: draw and return depth information of pixels in this frame
     *     in point cloud form
     */
    PointCloud::Ptr singleFrameDepth(const cv::Mat&rgb, const cv::Mat& depth);
    /*
     * drawDepth
     * @param const cv::Mat& rgb,the rgb image of frame
     * @param const cv::Mat& depth, the depth mat of frame (channel 1, 32-bit float)
     * @param const Pose& pose, the pose infomation of frame
     * function: combine depth infomation of this frame with older frames
     *     so current frame's pose infomation is needed
     *     depth infomation of tracked frames are all in point cloud form
     */
    void drawDepth(const cv::Mat& rgb, const cv::Mat& depth, const Pose& pose);
};

}

#endif
