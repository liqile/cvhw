#ifndef FEATURES_H
#define FEATURES_H

#include "ORBextractor.h"
#include "parameter.h"
#include "MapPoint.h"
#include "Pose.h"
#include <vector>
#include "ORBVocabulary.h"

namespace lqlslam{

class MapPoint;

struct Feature {
        /*
         * undistort key point
         */
        cv::KeyPoint keyPoint;
        /*
         * colum in right image
         */
        float right;
        /*
         * depth in current frame
         */
        float depth;

        /*
         * map point
         */
        MapPoint* mapPoint;

        /*
         * if the feature is an outlier during a tracking
         */
        bool isOutlier;

        /*
         * unProject
         * @param const Pose &pose, pose of camera
         * @param cv::Mat& p, store un project 3d coordinate when return
         * function: unproject the pixel to world system
         *         if depth is valid, do the unproject and return true
         *         otherwise return false
         */
        bool unProject(const Pose &pose, cv::Mat &p);

};

struct Features {
    public:
    /*
     * raw orb keypoints
     */
    std::vector<cv::KeyPoint> rawKeyPoints;

    /*
     * undistored keypoints
     */
    std::vector<Feature> keyPoints;

    /*
     * orb descriptors of keypoints
     */
    cv::Mat descriptors;

    /*
     * number of keypoints
     */
    int keyPointsNum;

    /*
     * index of features in each grid
     *         with x-coordinate first
     */
    std::vector<int> grid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    /*
     * bag of words vector
     */
    DBoW2::BowVector bowVec;

    /*
     * feature vector of bow
     */
    DBoW2::FeatureVector featVec;

    /*
     * undistort
     * function: undistort keypoints
     */
    void undistort();

    /*
     * getSteroMatch
     * function: for each keypoint, acquire its depth and compute
     *         colum of right img
     */
    void getSteroMatch(const cv::Mat& depth);

    /*
     * getGridPos
     * @param cv::KeyPoint& kp, undistort keypoint
     * @param int& posX, store x of grid when return
     * @param int& posY, store y of grid when return
     * function: check whether a keypoint is in a grid
     *         if it is in a grid, then return true, and fill
     *         posX and posY
     *         otherwise, return false
     */
    bool getGridPos(const cv::KeyPoint& kp, int& posX, int& posY);

    /*
     * assignFeaturesToGrid
     * function: assign keypoints in each level to
     *         grids on that level
     */
    void assignFeaturesToGrid();

    /*
     * computeBow
     * function: compute bag of words
     */
    void computeBow();

    Features(cv::Mat& image, const cv::Mat& depth);

    /*
     * getFeaturesInArea
     * @param const float& x, search center
     * @param const float& y, search center
     * @param const float& r, search area (max dx , max dy)
     * @param int minLevel, min level
     * @param int maxLevel, max level, -1 means top level
     * function: search features in area, with level in [minLevel, maxLevel]
     *           return index of Feature in features
     */
    vector<int> getFeaturesInArea(const float& x, const float& y, const float& r, const int minLevel, const int maxLevel = -1);
};

}

#endif
