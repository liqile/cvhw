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
    bool needUpdate;
    /*
     * min posible distance
     */
    float minDis;
    /*
     * max posible distance
     */
    float maxDis;
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

    /*
     * updateDescriptor
     * function: update descriptor
     */
    void updateDescriptor();

    /*
     * addObservation
     * @param cv::Mat& pos, pos of map point in world system
     * @param Frame* keyFrame, keyframe observes this point
     * @param int index, the index of feature in keyframe matching this point
     * function: add observation record with keyframe and index in this map point
     */
    void addObservation(const cv::Mat& pos, Frame* keyFrame, int index);

    /*
     * firstKFId
     * function : return id of ref kf
     */
    int firstKFId();
};

struct Marker {
    /*
     * current frame id matched in tracking
     * -1 for not visited
     */
    int trackMatchedFrame;
    /*
     * current frame id matched in local map
     * -1 for not visited
     */
    int mapMatchedFrame;
    /*
     * setTrackMatchedFrame
     * @param int id, id of current frame, -1 for no such frame
     */
    void setTrackMatchedFrame(int id);

    /*
     * setMapMatchedFrame
     * @param int id, id of current frame, -1 for no such frame
     */
    void setMapMatchedFrame(int id);

    Marker();
};

struct MapPoint {
    /*
     * whether map point is good
     */
    bool good;
    /*
     * id of next map point
     */
    static int nextId;
    /*
     * id of map point
     */
    int pointId;

    /*
     * pos of mappoint in word coordinate system
     */
    cv::Mat pos;

    /*
     * observation information of this map point
     */
    Observation observation;
    /*
     * log of tracking
     */
    Marker trackLog;
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

    /*
     * getDiscriptor
     * function: return a suitable discriptor of mappoint
     *         a mappoint can be observed by more than one keyframes
     *         so there may be more than one keypoint discriptors
     */
    cv::Mat getDiscriptor();

    /*
     * getOctave
     * @param float dis, distance of mappoint and camera center
     * function: compute and return a suitable octave for
     *         key point search
     */
    int getOctave(float dis);

    /*
     * getMinDis
     */
    float getMinDis();

    /*
     * getMaxDis
     */
    float getMaxDis();

    /*
     * merge
     * @param MapPoint* p, a map point
     * function: merge observations in p to current mappoint
     */
    void merge(MapPoint* p);

    /*
     * destroy
     * function: destroy this map point
     */
    void destroy();

    /*
     * merge
     * @param MapPoint* p1,
     * @param MapPoint* p2,
     * function: merge observation of p1 and p2 to one mappoint
     *         return the mappoint remained
     */
    static MapPoint* merge(MapPoint* p1, MapPoint* p2);

};

}

#endif
