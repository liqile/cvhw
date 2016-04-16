#ifndef MAP_POINT_H
#define MAP_POINT_H

#include "Frame.h"
#include "LocalMap.h"
#include "Pose.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <map>

namespace lqlslam {

class Frame;
class LocalMap;

struct Observation {
    /*
     * descriptor of mappoint
     */
    cv::Mat descriptor;

    /*
     * reference keyframe
     */
    Frame* refKF;

    /*
     * pairs of keyframes and corresbonding feature indices
     */
    std::map<Frame*, int> observations;

    /*
     * Observation
     * function: constructor
     */
    Observation();
};

struct MapPoint {
    /*
     * pos of mappoint in word coordinate system
     */
    cv::Mat pos;

    /*
     * observation information of this map point
     */
    Observation observation;

    /*
     * MapPoint
     * @param const cv::Mat& pos, the pos of point in word system
     * @param Frame* keyFrame, keyframe generate the point
     * @param int index, corresbonding feature index in keyframe
     * function: constructor
     */
    MapPoint(const cv::Mat& pos, Frame* keyFrame, int index);

    /*
     * addObservation
     * @param Frame* keyFrame, keyframe observes this point
     * @param int index, the index of feature in keyframe matching this point
     * function: add observation record with keyframe and index in this map point
     */
    void addObservation(Frame* keyFrame, int index);

    /*
     * getCamPose
     * @param const Pose& pose, pose of camera
     * function: compute and return the coordinate of mappoint under the
     *         system of camera
     */
    cv::Mat getCamPos(const Pose& pose);

    /*
     * reProject
     * @param const Pose& pose, the pose of camera
     * @param float& u, store column when return
     * @param float& v, store row when return
     * @param float& invz, store idepth when return
     * @param float& ur, store u-value in right camera when return (for rgbd, the right camera
     *         is a virtual camera)
     * function: reproject mappoint to a camera
     *         return true if invz > 0 and the u, v in the image( undistort boundary)
     */
    bool reProject(const Pose& pose, float& u, float& v, float& invz, float& ur);

    cv::Mat getDiscriptor();
};

}

#endif
