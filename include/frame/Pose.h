#ifndef POSE_H
#define POSE_H
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
namespace lqlslam {

struct Pose {
        /*
         * mTcw = [ mRcw mtcw]
         *        [    0  1  ]
         */
        cv::Mat mTcw;
        /*
         * mRcw = [xe, ye, ze]T
         * xe : camera's x direction in word coordinate system
         */
        cv::Mat mRcw;
        /*
         * co->wo in camera system
         */
        cv::Mat mtcw;
        /*
         * mRwc = [xe, ye, ze]
         * xe : camera's x direction in word coordinate system
         * wp = mRwc * lp + mOw
         */
        cv::Mat mRwc;
        /*
         * camera center in word coordinate system
         * wp = mRwc * lp + mOw
         */
        cv::Mat mOw;
        Pose();
        /*
         * Pose
         * @param const Pose& pose,
         * function: copy
         */
        Pose(const Pose& pose);
        /*
         * setPose
         * @param const cv::Mat& mTcw, pose information, mTcw period, elements are float type
         * function: set pose information
         */
        void setPose(const cv::Mat& mTcw);
        /*
         * setPoseWC
         * @param const cv::Mat& Rcw, elements are float type
         * @param const cv::Mat& tcw,
         * function: set pose information
         */
        void setPose(const cv::Mat& Rcw, const cv::Mat& tcw);
        /*
         * toWorld
         * @param const cv::Mat& localPos
         * function: a const function, compute and return worldPos of point
         */
        cv::Mat toWorld(const cv::Mat& localPos) const;
        /*
         * toLocal
         * @param const cv::Mat& worldPos
         * function: a const function, compute and return local pos of point
         */
        cv::Mat toLocal(const cv::Mat& worldPos) const;
        /*
         * computeEpipole
         * @param const Pose& p1, pose of first frame
         * @param float& ex, store ex of epipole on second frame (this)
         * @param float& ey, store ey of epipole on second frame (this)
         * function: compute epipole of second frame,
         *         here, [this] ptr point to the pose of second frame
         */
        void computeEpipole(const Pose& p1, float& ex, float& ey) const;
        /*
         * getF12
         * @param const Pose& p1, pose of first frame
         * function: compute and return F12,
         *         here, [this] ptr point to the pose of second frame
         */
        cv::Mat getF12(const Pose& p1) const;
        /*
         * checkPoint
         * @param const cv::Mat& pos, pos of 3d point in world system
         * @param const cv::KeyPoint& k, key point
         * function: check pos whether in front of camera,
         *         and check the distance of projection key point
         *         if the pos is valid, return true, otherwise return false
         */
        bool checkPoint(const cv::Mat& pos, const cv::KeyPoint& k) const;
        /*
         * distance
         * @param const cv::Mat& wp, 3d coordinate in world system
         * function: get the distance of wp and camera center
         */
        float distance(const cv::Mat& wp) const;
        /*
         * skewSymmetricMatrix
         * @param const cv::Mat& v, a 3d vector
         * function: return skey symmetric matrix of v
         */
        static cv::Mat skewSymmetricMatrix(const cv::Mat& v);
        /*
         * getEpipolarLine
         * @param const cv::KeyPoint& kp1, keypoint in frame1
         * @param const cv::Mat& F12, fundamental matrix of frame1 and frame2
         * @param float& a, line: ax + by + c = 0 on second frame
         * @param float& b, line: ax + by + c = 0 on second frame
         * @param float& c, line: ax + by + c = 0 on second frame
         * function: get eppipolar line on second frame, line: Ax + by + c = 0
         */
        static void getEpipolarLine(const cv::KeyPoint &kp1, const cv::Mat &F12, float& a, float& b, float& c);
        /*
         * checkDistEpipolarLine
         * @param const cv::KeyPoint& kp1, keypoint in frame1
         * @param const cv::KeyPoint& kp2, keypoint in frame2
         * @param const cv::Mat& F12, fundamental matrix of frame1 and frame2
         * function: check whether kp1 and kp2 satisify eppipolar constraint
         */
        static bool checkDistEpipolarLine(const cv::KeyPoint &kp1,const cv::KeyPoint &kp2,const cv::Mat &F12);
        /*
         * triangular
         * @param const Pose& p1, pose of frame1 (current frame)
         * @param const Pose& p2, pose of frame2 (last frame)
         * @param const cv::KeyPoint& k1, key point in frame1
         * @param const cv::KeyPoint& k2, key point in frame2
         * function: given a match of k1 and k2. compute 3d-coordinate
         *         by triangulation and return. if the coordinate isn't valid,
         *         return an empty mat
         */
        static cv::Mat triangular(const Pose& p1, const Pose& p2, const cv::KeyPoint& k1, const cv::KeyPoint& k2);

        static cv::Mat triangularInit(const Pose& p1, const Pose& p2, const cv::KeyPoint& k1, const cv::KeyPoint& k2);
};

}

#endif
