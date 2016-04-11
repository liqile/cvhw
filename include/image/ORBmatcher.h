#ifndef ORB_MATCHER_H
#define ORB_MATCHER_H

#include "Features.h"
#include "Frame.h"
#include "TrackingDrawer.h"

namespace lqlslam {

#define TH_HIGH 100
#define TH_LOW 50
#define HISTO_LENGTH 30

class ORBmatcher {
    private:
    /*
     * current frame
     */
    Frame* currFrame;
    /*
     * last frame
     */
    Frame* lastFrame;
    /*
     * feature set of curr frame
     */
    Features* curr;
    /*
     * pose of curr frame
     */
    Pose pose;
    /*
     * count number of matches in the end
     */
    int matches;
    /*
     * 1.0 / HISTO_LENGTH
     */
    float factor;
    /*
     * for each keypoint match, compute the dis-angle and
     * record corresbonding idx of the match in current frame
     * the dis-angle range is 0~360 and is split to HISTO_LENGTH parts
     */
    vector<int> rotHist[HISTO_LENGTH];

    int bestMatch(const cv::Mat& desLast, float ur, const std::vector<int>& vidxCurr, float radius, int& bestIdxCurr);

    /*
     * matchInit
     * function: init matcher
     */
    void matchInit();

    /*
     * getAngle
     * @param const Feature& featureCurr, feature of curr frame
     * @param const Feature& featureLast, feature of last frame
     * function: compute dis-angle of current feature and last feature
     *         return corresponding index in rotHist
     */
    int getAngle(const Feature& featureCurr, const Feature& featureLast);
    /*
     * computeThreeMaximima
     * @param int& ind1, idx of max{rotHist[idx].size()}
     * @param int& ind2, idx of second max {rotHist[idx].size()}
     * @param int& ind3, idx of third max {rotHist[idx].size()}
     * function: get indices of 3 max {rotHist[idx].size()}
     */
    void computeThreeMaxima(int& ind1, int&ind2, int&ind3);
    /*
     * filterByRot
     * function: filter matches preserving 3 heighest appreance dis-angle of keypoint pairs
     */
    void filterByRot();
    public:
    /*
     * ORBmatcher
     * @param Frame* currFrame, current frame, whose keypoints will be matched
     * function: constructor
     */
    ORBmatcher(Frame* currFrame);
    /*
     * descriptorDis
     * @param const cv::Mat& a
     * @param const cv::Mat& b
     * function: compute and return distance of
     *         two orb descriptor a and b
     */
    static int descriptorDis(const cv::Mat& a, const cv::Mat& b);
    /*
     * searchByProject
     * @param Frame* lastFrame, last frame
     * @param float th, the search range, in level i, the search range is th * sf^i
     *         sf is the scale factor of pyramid, 1.2 as defualt
     * function: match keypoints between current Frame and last Frame
     *         by projecting mappoints from lastFrame to currFrame
     *         after matching,  the curr (Features type) will contain corresponding mappoint information
     *         the return value is the number of matches found
     */
    int searchByProject(Frame* lastFrame, float th);
    /*
     * searchByTriangular
     * @param Frame* secondFrame, second frame, current frame as first frame
     * @param vector<pair<int, int> >& matches, store matches of keypoints index of two frames
     * function: search unmatched keypoint matches between first and second frames
     *         pair of matched indices will be stored in matches
     */
    int searchByTriangular(Frame* secondFrame, vector<pair<int, int> >& matches);
};

}

#endif
