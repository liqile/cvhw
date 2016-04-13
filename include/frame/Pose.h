#ifndef POSE_H
#define POSE_H
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
namespace lqlslam {

struct Pose {
        /*
         * mTcw = [ mRcw mtcw]
         *               [     0          1  ]
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
        /*
         * setPose
         * @param const cv::Mat& mTcw, pose information, mTcw period
         * function: set pose information
         */
        void setPose(const cv::Mat& mTcw);
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
        void computeEpipole(const Pose& p1, float& ex, float& ey);
        /*
         * getF12
         * @param const Pose& p1, pose of first frame
         * function: compute and return F12,
         *         here, [this] ptr point to the pose of second frame
         */
        cv::Mat getF12(const Pose& p1);
        /*
         * skewSymmetricMatrix
         * @param const cv::Mat& v, a 3d vector
         * function: return skey symmetric matrix of v
         */
        static cv::Mat skewSymmetricMatrix(const cv::Mat& v);
        /*
         * checkDistEpipolarLine
         * @param const cv::KeyPoint& kp1, keypoint in frame1
         * @param const cv::KeyPoint& kp2, keypoint in frame2
         * @param const cv::Mat& F12, fundamental matrix of frame1 and frame2
         * function: check whether kp1 and kp2 satisify eppipolar constraint
         */
        static bool checkDistEpipolarLine(const cv::KeyPoint &kp1,const cv::KeyPoint &kp2,const cv::Mat &F12);
};

}

#endif