#include "ORBmatcher.h"
#include "drawers.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

namespace lqlslam {

bool unmatched(const Feature& feature) {
    return (feature.mapPoint != NULL);
}

bool matched(const Feature& feature) {
    return (feature.mapPoint == NULL);
}

int ORBmatcher::bestMatch(const cv::Mat& desLast, float ur, const std::vector<int>& vidxCurr, float radius, int& bestIdxCurr) {
    int bestDist = 256;
    bestIdxCurr = -1;
    for (int j = 0; j < vidxCurr.size(); j++) {
        int idxCurr = vidxCurr[j];
        const Feature& featureCurr = curr->keyPoints[idxCurr];
        if (!mark->canMatch(featureCurr)) {
            continue;
        }
        /*
        if (featureCurr.mapPoint != NULL) {
            //todo check observations
            continue;
        }
        if (featureCurr.right > 0 && fabs(ur - featureCurr.right) > radius) {
            continue;
        }*/
        int dist = descriptorDis (curr->descriptors.row(idxCurr), desLast);
        if (dist < bestDist) {
            bestDist = dist;
            bestIdxCurr = idxCurr;
        }
    }
    return bestDist;
}

void ORBmatcher::matchInit() {
    matches = 0;
    for (int i = 0; i < HISTO_LENGTH; i++) {
        rotHist[i].clear();
    }
    factor = 1.0 / HISTO_LENGTH;
    counter->reset(curr, lastFrame->features);
}

int ORBmatcher::getAngle (const Feature &featureCurr, const Feature &featureLast) {
    float rot = featureLast.keyPoint.angle - featureCurr.keyPoint.angle;
    if (rot < 0) {
        rot += 360;
    }
    int bin = round(rot * factor);
    if (bin == HISTO_LENGTH) {
        bin = 0;
    }
    assert(bin >= 0 && bin < HISTO_LENGTH);
    return bin;
}

void ORBmatcher::computeThreeMaxima (int &ind1, int &ind2, int &ind3) {
    int max1=0;
    int max2=0;
    int max3=0;
    for(int i=0; i<HISTO_LENGTH; i++) {
        const int s = rotHist[i].size();
        if(s>max1) {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2) {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3) {
            max3=s;
            ind3=i;
        }
    }
    if(max2<0.1f*(float)max1) {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1) {
        ind3=-1;
    }
}

void ORBmatcher::filterByRot() {
    int ind1=-1;
    int ind2=-1;
    int ind3=-1;

    computeThreeMaxima(ind1,ind2,ind3);

    for(int i=0; i<HISTO_LENGTH; i++)
    {
        if(i!=ind1 && i!=ind2 && i!=ind3)
        {
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                //curr->keyPoints[rotHist[i][j]].mapPoint=static_cast<MapPoint*>(NULL);
                counter->reduceMatch(rotHist[i][j]);
                matches--;
            }
        }
    }
}

int ORBmatcher::disMatch(const cv::Mat& d1, const Indices& indices2, int& idx2) {
    const Features* feat2 = lastFrame->features;
    int bestDis = TH_LOW;
    idx2 = -1;
    for (int it = 0; it < indices2.size(); it++) {
        int i = indices2[it];
        if (counter->currIdx[i] != -1) {
            continue;
        }
        const Feature& f2 = feat2->keyPoints[i];
        if (!checker->need2(f2)) {
            continue;
        }
        const cv::Mat& d2 = feat2->descriptors.row(i);
        int dis = descriptorDis(d1, d2);
        if (dis < bestDis) {
            bestDis = dis;
            idx2 = i;
        }
    }
    return bestDis;
}

void ORBmatcher::searchMatches(const Indices& idx1, const Indices& idx2) {
    int count = 0;
    for (int it = 0; it < idx1.size(); it++) {
        int i = idx1[it];
        const Feature& f1 = this->currFrame->features->keyPoints[i];
        if (!checker->need1(f1)) {
            continue;
        }
        const cv::Mat& d1 = currFrame->features->descriptors.row(i);
        int j, dis;
        dis = disMatch(d1, idx2, j);
        if (dis < TH_LOW) {
            const Feature& f2 = this->lastFrame->features->keyPoints[j];
            if (!checker->check(f1, f2)) {
                continue;
            }
            count ++;
            int bin = getAngle(f1, f2);
            rotHist[bin].push_back(i);
            counter->addMatch (i, j);
        }
    }
    //cout << "find: " << count << endl;
}

ORBmatcher::ORBmatcher(Frame* currFrame) {
    this->currFrame = currFrame;
    this->curr = currFrame->features;
    this->pose = currFrame->pose;
    for (int i = 0; i < HISTO_LENGTH; i++) {
        rotHist[i].reserve(500);
    }
}

int ORBmatcher::descriptorDis(const cv::Mat &a, const cv::Mat &b) {
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

int ORBmatcher::searchByProject(Frame* lastFrame, float th, bool usePointDes) {
    //th *= 2;
    const Features* last = lastFrame->features;
    this->lastFrame = lastFrame;
    matchInit();
    int pointNum = 0;
    int unvisitNum = 0;
    for (int i = 0; i < last->keyPointsNum; i++) {
        //int idxLast = i;
        const Feature& featureLast = last->keyPoints[i];
        MapPoint* point = featureLast.mapPoint;
        if (point == NULL) {
            continue;
        }
        if (!point->good) {
            continue;
        }
        /*
        if (point->trackLog.trackMatchedFrame == currFrame->frameId) {
            continue;
        }*/
        pointNum ++;
        if (!mark->unVisited(point)) {
            continue;
        }
        mark->mark(point);
        unvisitNum ++;
        float u, v, invzc, ur;
        if (!point->reProject (pose, u, v, invzc, ur)) {
            continue;
        }
        counter->addLastProjectNum();
        int lastOctave = featureLast.keyPoint.octave;
        //cout <<"level: "<<lastOctave<<" radius: " <<radius<<endl;
        //const vector<int>& vidxCurr = curr->getFeaturesInArea (u, v, radius, lastOctave - 1, lastOctave + 1);
        // u = featureLast.keyPoint.pt.x;
        // v = featureLast.keyPoint.pt.y;
        int minOctave;
        int maxOctave;
        cv::Mat desLast;
        float radius;
        if (!usePointDes) {
            minOctave = lastOctave - 1;
            maxOctave = lastOctave + 1;
            desLast = last->descriptors.row(i);
            radius = th * extract->scaleFactor[lastOctave];
        } else {
            desLast = point->getDiscriptor();
            float dis = pose.distance(point->pos);
            maxOctave = point->getOctave(dis);
            minOctave = maxOctave - 1;
            if (minOctave < 0) {
                minOctave = 0;
            }
            radius = th * extract->scaleFactor[minOctave] * sqrt(extract->fScaleFactor);
        }
        const vector<int>& vidxCurr = curr->getFeaturesInArea (u, v, radius, minOctave, maxOctave);
        if (vidxCurr.size() == 0) {
            continue;
        }

        //cv::Mat desLast = last->descriptors.row(i);

        int bestIdxCurr;
        int bestDist = bestMatch(desLast, ur, vidxCurr, radius, bestIdxCurr);
        if (bestDist <= TH_HIGH) {
            const Feature& featureCurr = curr->keyPoints[bestIdxCurr];
            //featureCurr.mapPoint = point;
            int bin = getAngle(featureCurr, featureLast);
            rotHist[bin].push_back (bestIdxCurr);
            matches ++;
            counter->addMatch(bestIdxCurr, i);
        }
    }
    cout << "[tracking] current frame id: " << currFrame->frameId << endl;
    cout << "[tracking] last frame map point num: " << pointNum << endl;
    cout << "[tracking] last frame unvisit point num: " << unvisitNum << endl;
    filterByRot();
#if 0
    //counter->drawTrackingMatches(currFrame, lastFrame);
    counter->drawMatches(currFrame, lastFrame);
#endif
    return matches;
}

void ORBmatcher::searchByTriangular(Frame *secondFrame) {
    Frame* firstFrame = currFrame;
    this->lastFrame = secondFrame;
    counter = &mappingMatcherCounter;
    //matches.clear();
    const DBoW2::FeatureVector &fv1 = firstFrame->features->featVec;
    const DBoW2::FeatureVector &fv2 = secondFrame->features->featVec;
    matchInit();
    DBoW2::FeatureVector::const_iterator it1 = fv1.begin();
    DBoW2::FeatureVector::const_iterator it2 = fv2.begin();
    checker = new TChecker(firstFrame, secondFrame);
    while (it1 != fv1.end() && it2 != fv2.end()) {
        int class1 = it1->first;
        int class2 = it2->first;
        if (class1 == class2) {
            const Indices& idx1 = it1->second;
            const Indices& idx2 = it2->second;
            //cout << "curr class: " << class1 << " size: " << idx1.size() << endl;
            //cout << "last class: " << class2 << " size: " << idx2.size() << endl << endl;
            searchMatches(idx1, idx2);
            it1 ++;
            it2 ++;
        } else {
            if (class1 < class2) {
                it1 = fv1.lower_bound(class2);
            } else {
                it2 = fv2.lower_bound(class1);
            }
        }
    }
    filterByRot();
#if 0
    matches.clear();
    for (int i = 0; i < counter->lastIdx.size(); i++) {
        int idx1 = i;
        int idx2 = counter->lastIdx[i];
        matches.push_back(make_pair(idx1, idx2));
    }
#endif
#if 0
    counter->drawMatches(currFrame, lastFrame);
#endif
    //return matches.size();
}
#if 0
void ORBmatcher::setTrackPoint(Frame* lastFrame) {
    for (int i = 0; i < curr->keyPointsNum; i++) {
        int lastIdx = trackingMatcherCounter.lastIdx[i];
        if (lastIdx == -1) {
            continue;
        }
        Feature& fl = lastFrame->features->keyPoints[lastIdx];
        Feature& fc = curr->keyPoints[i];
        fc.mapPoint = fl.mapPoint;
    }
}
#endif
void ORBmatcher::searchLastFrame(Frame *lastFrame, float th) {
    mark = new TrackMark(currFrame, lastFrame);
    counter = &trackingMatcherCounter;
    searchByProject(lastFrame, th, false);
    delete mark;
}

void ORBmatcher::searchKeyFrame(Frame *keyFrame, float th) {
    mark = new TrackMark(currFrame, keyFrame);
    counter = &trackingMatcherCounter;
    searchByProject(keyFrame, th, true);
    delete mark;
}

void ORBmatcher::searchForFuse(Frame *keyFrame, float th) {
    mark = new FuseMark(currFrame, keyFrame);
    counter = &mappingMatcherCounter;
    searchByProject(keyFrame, th, true);
    delete mark;
}

void ORBmatcher::searchForInit(Frame *frame2, vector<cv::DMatch>& matches) {
    Frame* frame1 = currFrame;
    cv::BFMatcher matcher(cv::NORM_HAMMING2);
    matches.clear();
    vector<cv::DMatch> allMatches;
    matcher.match(frame1->features->descriptors, frame2->features->descriptors, allMatches);
    float maxDis = 0;
    float minDis = 128;
    for (int i = 0; i < allMatches.size(); i++) {
        const cv::DMatch& match = allMatches[i];
        if (maxDis < match.distance) {
            maxDis = match.distance;
        }
        if (minDis > match.distance) {
            minDis = match.distance;
        }
    }
    for (int i = 0; i < allMatches.size(); i++) {
        const cv::DMatch& match = allMatches[i];
        if (match.distance < maxDis / 2) {
            matches.push_back(match);
        }
    }
    cout << maxDis << " :: " << minDis << endl;
}

cv::Mat ORBmatcher::getFun12() {
    TChecker* c = (TChecker*) checker;
    return c->fun12;
}

}
