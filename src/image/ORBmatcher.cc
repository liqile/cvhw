#include "ORBmatcher.h"
#include "drawers.h"

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
                curr->keyPoints[rotHist[i][j]].mapPoint=static_cast<MapPoint*>(NULL);
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
    for (int i = 0; i < indices2.size(); i++) {
        if (counter->currIdx[i] != -1) {
            continue;
        }
        const Feature& f2 = feat2->keyPoints[i];
        if (!checker->need(f2)) {
            continue;
        }
        const cv::Mat& d2 = feat2->descriptor.row(i);
        int dis = descriptorDis(d1, d2);
        if (dis < bestDis) {
            bestDis = dis;
            idx2 = i;
        }
    }
    return bestDis;
}

void ORBmatcher::searchMatches(const Indices& idx1, const Indices& idx2) {
    for (int i = 0; i < idx1.size(); i++) {
        const Feature& f1 = this->currFrame->features->keyPoints[i];
        if (!checker->need(f1)) {
            continue;
        }
        const cv::Mat& d1 = currFrame->features->descriptors.row(i);
        int j, dis;
        dis = disMatch(d1, idx2, j);
        if (dis < TH_LOW) {
            const Feature& f2 = this->lastFrame->features->keyPoints(j);
            int bin = getAngle(f1, f2);
            rotHist.push_back(bin, i);
            counter->addMatch (i, j);
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
    counter = &trackingMatcherCounter;
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
        counter->addLastProjectNum();
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
            counter->addMatch(bestIdxCurr, i);
        }
    }
    filterByRot();
#if DEBUG_MATCHER
    counter->drawMatches(currFrame, lastFrame);
#endif
    return matches;
}

int ORBmatcher::searchByTriangular(Frame *secondFrame, Matches &matches) {
    Frame* firstFrame = currFrame;
    this->lastFrame = secondFrame;
    counter = &mappingMatcherCounter;
    matches.clear();
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
            searchMatches(idx1, idx2);
        } else {
            if (class1 < class2) {
                it1 = fv1.lower_bound(class2);
            } else {
                it2 = fv2.lower_bound(class1);
            }
        }
    }
    fillterByRot();
    matches.clear();
    for (int i = 0; i < counter->lastIdx.size(); i++) {
        int idx1 = i;
        int idx2 = counter->lastIdx[i];
        matches.push_back(make_pair(idx1, idx2));
    }
    return matches.size();
}

}
