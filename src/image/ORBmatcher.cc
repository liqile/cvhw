#include "ORBmatcher.h"
#include "drawers.h"

namespace lqlslam {

int ORBmatcher::bestMatch(const cv::Mat& desLast, float ur, const std::vector<int>& vidxCurr, float radius, int& bestIdxCurr) {
    int bestDist = 256;
    bestIdxCurr = -1;
    for (int j = 0; j < vidxCurr.size(); j++) {
        int idxCurr = vidxCurr[j];
        Feature& featureCurr = curr->keyPoints[idxCurr];
        if (featureCurr.mapPoint != NULL) {
            //todo check observations
            continue;
        }
        /*
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
    trackingMatcherCounter.reset(curr, lastFrame->features);
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
                curr->keyPoints[rotHist[i][j]].mapPoint=static_cast<MapPoint*>(NULL);
                trackingMatcherCounter.reduceMatch(rotHist[i][j]);
                matches--;
            }
        }
    }
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

int ORBmatcher::searchByProject(Frame* lastFrame, float th) {
    //th *= 2;
    const Features* last = lastFrame->features;
    this->lastFrame = lastFrame;
    matchInit();
    for (int i = 0; i < last->keyPointsNum; i++) {
        //int idxLast = i;
        cv::Mat desLast = last->descriptors.row(i);
        const Feature& featureLast = last->keyPoints[i];
        MapPoint* point = featureLast.mapPoint;
        if (point == NULL) {
            continue;
        }
        float u, v, invzc, ur;
        if (!point->reProject (pose, u, v, invzc, ur)) {
            continue;
        }
        trackingMatcherCounter.addLastProjectNum();
        int lastOctave = featureLast.keyPoint.octave;
        float radius = th * extract->scaleFactor[lastOctave];
        //cout <<"level: "<<lastOctave<<" radius: " <<radius<<endl;
        //const vector<int>& vidxCurr = curr->getFeaturesInArea (u, v, radius, lastOctave - 1, lastOctave + 1);
         u = featureLast.keyPoint.pt.x;
         v = featureLast.keyPoint.pt.y;
        const vector<int>& vidxCurr = curr->getFeaturesInArea (u, v, radius, lastOctave - 1, lastOctave + 1);
        if (vidxCurr.size() == 0) {
            continue;
        }
        int bestIdxCurr;
        int bestDist = bestMatch(desLast, ur, vidxCurr, radius, bestIdxCurr);
        if (bestDist <= TH_HIGH) {
            Feature& featureCurr = curr->keyPoints[bestIdxCurr];
            featureCurr.mapPoint = point;
            int bin = getAngle(featureCurr, featureLast);
            rotHist[bin].push_back (bestIdxCurr);
            matches ++;
            trackingMatcherCounter.addMatch(bestIdxCurr, i);
        }
    }
    filterByRot();
#if DEBUG_MATCHER
    trackingMatcherCounter.drawMatches(currFrame, lastFrame);
#endif
    return matches;
}

int ORBmatcher::searchByTriangular(Frame *secondFrame, vector<pair<int, int> > &matches) {
    Frame* firstFrame = currFrame;
    matches.clear();
}

}
