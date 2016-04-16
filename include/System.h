#ifndef SYSTEM_H
#define SYSTEM_H

#include "Frame.h"
#include "Map.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "Tracking.h"

namespace lqlslam {

class System {

    private:

    int state;

    /*
     * tracker
     */
    Tracking* tracker;


    #if DEBUG_MATCHER
    /*
     * last Frame
     */
    Frame* lastKeyFrame;
    #endif

    /*
     * initialize
     * @param Frame* frame, the first frame ready to initialize
     * function: regard frame as first frame in the slam system,
     *     initialize map
     */
    void initialize(Frame* frame);
    /*
     * needKeyFrame
     * @param Frame* frame, current frame
     * @param int points, number of map points tracked
     * function: check if current frame would become a key frame
     */
    bool needKeyFrame(Frame* frame, int points);
    /*
     * createNewKeyFrame
     * @param Frame* frame, current frame
     * function: create a new key frame and send it to map
     */
    void createNewKeyFrame(Frame* frame);
    public:

    /*
     * map
     */
    Map* map;

    public:

    /*
     * System
     * @param const string& fileName, file contains parameters
     * function: constructor
     */
    System(const string& fileName);

    /*
     * track
     * @param const cv::Mat& img, the gray image
     * @param const cv::Mat& depth, the depth mat (32F)
     * @param const double& timestamp, the timestamp of image
     * function: track img, return pose information
     */
    Pose track(const cv::Mat& img, const cv::Mat& depth, const double& timestamp);

    /*
     * test
     * function: some test
     */
    void test(const cv::Mat& image);

};

void readImage(const string& imgName, const string& depthName, cv::Mat& img, cv::Mat& depth, cv::Mat& rgb);
/*
void trackImage();

void testSystem();

void testSystem2();
*/
void testSystem3();

void readFileNames(const string& path, vector<string>& names);

}

#endif
