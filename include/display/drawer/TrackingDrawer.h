#ifndef TRACKING_DEPTH_DRAWER_H
#define TRACKING_DEPTH_DRAWER_H

#include "Frame.h"
#include "Features.h"
#include "display.h"

namespace lqlslam {

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
