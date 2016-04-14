#ifndef FRAME_H
#define FRAME_H

#include "ORBextractor.h"
#include "parameter.h"
#include "Features.h"
#include <vector>
#include "Pose.h"
using namespace std;

namespace lqlslam {

void testFrameFeature (ORBextractor* extractor, const cv::Mat& imGray);

class MapPoint;
class Features;

struct RawData {
        cv::Mat img;
        cv::Mat depth;
        double timeStamp;
        RawData(const cv::Mat& image, const cv::Mat& depthInfo, const double& timeStamp);
};

class Frame {

        protected:

        Frame(){}

        private:

        /*
         * next id of frame
         */
        static int nextId;

        /*
         * next id of keyframe
         */
        static int nextKFId;

        /*
         * whether current frame becomes a keyframe
         */
        bool isKeyFrame;

        public:

        /*
         * id of current frame
         */
        int frameId;

        /*
         * id of current key frame
         */
        int keyFrameId;

        /*
         * gray image and timestamp
         */
        RawData* rawData;

        /*
         * features
         */
        Features* features;

        /*
         * pose of camera
         */
        Pose pose;

        /*
         * Frame
         * @param const cv::Mat& imGray
         * @param const cv::Mat& depth
         * @param const double& timeStamp
         * @param ORBextractor* extractor
         */
        Frame(
                const cv::Mat &imGray,
                const cv::Mat& depth,
                const double &timeStamp
        );

        /*
         * setPose
         * @param const cv::Mat& mTcw,
         * function: set pose
         */
        void setPose(const cv::Mat& mTcw);

        /*
         * unProject
         * @param const int& i, ith features
         * @param cv::Mat& p, store ith features' coordinate
         *         in word coordinate system after return
         * function: compute ith features' coordinate in word
         *         if ith feature has no depth, return false, otherwise
         *         return true, and store coordinate in p
         */
        bool unProject(const int& i, cv::Mat& p);

        /*
         * generate3dPoints
         * function: generate 3d points for keypoints
         *         if corresbonding depth information from sensor is valid
         *         for monocular, this function does nothing
         */
        void generate3dPoints();

        /*
         * becomeKeyframe
         * function: set isKeyframe as true;
         */
        void becomeKeyframe();

#if 0
        /*
         * reset
         * function: reset this keyframe as first keyframe in system
         *         set pose as I, generate map points
         */
        void reset();
        #endif
};

void testFrame();

}

#endif
