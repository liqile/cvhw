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
};

}

#endif
