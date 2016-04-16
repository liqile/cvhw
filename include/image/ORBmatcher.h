#ifndef ORB_MATCHER_H
#define ORB_MATCHER_H

#include "Features.h"
#include "Frame.h"
#include "TrackingDrawer.h"

namespace lqlslam {

#define TH_HIGH 100
#define TH_LOW 50
#define HISTO_LENGTH 30

typedef vector<unsigned int> Indices;
typedef vector<pair<int, int> > Matches;

typedef bool (*NeedMatch)(const Feature& feature);

bool unmatched(const Feature& feature);

bool matched(const Feature& feature);

struct Checker {
    /*
     * first frame, current frame
     */
    Frame* f1;
    /*
     * second frame, last frame
     */
    Frame* f2;
    /*
     * Checker
     * @param Frame* f1,
     * @param Frame* f2,
     * function: constructor
     */
    Checker(Frame* f1, Frame* f2) {
        this->f1 = f1;
        this->f2 = f2;
    }
    /*
     * need1
     * @param const Feature& f, feature in frame1
     * function: check whether feature f in frame1 needs a match
     */
    virtual bool need1(const Feature& f) {
        return (f.mapPoint != NULL);
    }
    /*
     * need2
     * @param const Feature& f, feature in frame2
     * function: check whether feature f in frame2 needs a match
     */
    virtual bool need2(const Feature& f) {
        return (f.mapPoint == NULL);
    }
    /*
     * check
     * @param const Feature& f1, feature in frame1
     * @param const Feature& f2, feature in frame2
     * function: check whether f1 and f2 is a valid match
     */
    virtual bool check(const Feature& f1, const Feature& f2) {
        return true;
    }
};

struct TChecker:public Checker {
    /*
     * ex of eppipolar
     */
    float ex;
    /*
     * ey of eppipolar
     */
    float ey;
    /*
     * fundamental matrix of f1 and f2
     */
    cv::Mat fun12;
    TChecker(Frame* f1, Frame* f2) : Checker(f1, f2) {
        f2->pose.computeEpipole (f1->pose, ex, ey);
        fun12 = f2->pose.getF12 (f1->pose);
    }
    bool need1(const Feature& f) {
#if ONLY_TRACKING
        return true;
#endif
        return true;
        return (f.mapPoint == NULL);
    }
    bool need2(const Feature& f) {
#if ONLY_TRACKING
        return true;
#endif
        return true;
        return (f.mapPoint == NULL);
    }

    /*
     * check
     * @param const Feature& f1, feature in frame1
     * @param const Feature& f2, feature in frame2
     * function: check whether f1 and f2 satisify epipole constraint
     */
    bool check(const Feature& f1, const Feature& f2) {
        const cv::KeyPoint& kp1= f1.keyPoint;
        const cv::KeyPoint& kp2 = f2.keyPoint;
        const float distex = ex - kp1.pt.x;
        const float distey = ey - kp2.pt.y;
        const int octave = kp2.octave;
        if (distex * distex + distey * distey < 100 * extract->scaleFactor[octave]) {
            return false;
        }
        return Pose::checkDistEpipolarLine (kp1, kp2, fun12);
    }
};

class ORBmatcher {
    private:
    /*
     * counter of matches
     */
    MatcherCounter* counter;
    /*
     * checker of matches
     */
    Checker* checker;
    /*
     * a function, check whether a Feature needs a match
     */
    //NeedMatch need;
    /*
     * current frame, also regarded as first frame
     */
    Frame* currFrame;
    /*
     * last frame, also regarded as second frame
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
    /*
     * disMatch
     * @param const cv::Mat& d1, descriptor of keypoint in first frame (curr frame)
     * @param const Indices& indices, idx of keypoints in last frame (second frame)
     * @param int& idx2, best match idx of keypoint in second frame
     * function: find best matches in idx2 with d1, return dis,
     *         index in second(last) frame stored in idx2
     */
    int disMatch(const cv::Mat& d1, const Indices& indices2, int& idx2);
    /*
     * searchMatches
     * @param const Indices& idx1, keypoints indices of first frame (current frame)
     * @param const Indices& idx2, keypoints indices of second frame (last frame)
     * function: search matches between idx1 and idx2, matches will be stored in
     *         tracking matcher counter
     */
    void searchMatches(const Indices& idx1, const Indices& idx2);
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
     *         trackingMatchCounter will count
     */
    int searchByProject(Frame* lastFrame, float th);
    /*
     * searchByTriangular
     * @param Frame* secondFrame, second frame, current frame as first frame
     * function: search unmatched keypoint matches between first and second frames
     *         pair of matched indices will be stored in matches
     *         return number of matches
     *         mappingMatchCounter will count
     */
    void searchByTriangular(Frame* secondFrame);//, vector<pair<int, int> >& matches);
};

}

#endif
