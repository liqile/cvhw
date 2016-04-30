#include "Features.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "Pose.h"
#include <iostream>
using namespace std;
namespace lqlslam{
void test(ORBextractor* extractor, const cv::Mat& image) {
    std::vector<cv::KeyPoint> keyPoints;
    cv::Mat descriptor;
    (*extractor) (image, cv::Mat(), keyPoints, descriptor);
    cout <<" test feature extract: " << keyPoints.size () << endl;
}
bool Feature::unProject (const Pose &pose, cv::Mat &p) {
    const float z = depth;
    if (z > 0) {
        const float u = keyPoint.pt.x;
        const float v = keyPoint.pt.y;
        const float x = (u - camera->cx) * z * camera->invFx;
        const float y = (v - camera->cy) * z * camera->invFy;
        cv::Mat x3DC = (cv::Mat_<float>(3,1) << x, y, z);
        p = pose.toWorld(x3DC);
        return true;
    } else {
        return false;
    }
}

void Features::undistort() {
    cv::Mat k = camera->k;
    cv::Mat distCoef = camera->distCoef;
    if(distCoef.at<float>(0)==0.0) {
            for (int i = 0; i < rawKeyPoints.size(); i++) {
                    Feature& kp = keyPoints[i];
                    kp.keyPoint = rawKeyPoints[i];
                    kp.depth = -1;
                    kp.right=  -1;
            }
            return;
    }
    cv::Mat mat(keyPointsNum,2,CV_32F);
    for(int i=0; i<keyPointsNum; i++) {
            mat.at<float>(i,0)=rawKeyPoints[i].pt.x;
            mat.at<float>(i,1)=rawKeyPoints[i].pt.y;
    }
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,k,distCoef,cv::Mat(),k);
    mat=mat.reshape(1);
    for(int i=0; i<keyPointsNum; i++) {
            cv::KeyPoint kp = rawKeyPoints[i];
            kp.pt.x=mat.at<float>(i,0);
            kp.pt.y=mat.at<float>(i,1);
            Feature& feature = keyPoints[i];
            feature.keyPoint=kp;
            feature.depth = -1;
            feature.right = -1;
    }
}

void Features::getSteroMatch (const cv::Mat& depth) {
    if (depth.empty()) {
        return;
    }
    for (int i = 0; i < keyPointsNum; i++) {
        const cv::KeyPoint& kp = rawKeyPoints[i];
        Feature& feature = keyPoints[i];
        const float& v = kp.pt.y;
        const float& u = kp.pt.x;
        const float d =depth.at<float>(v, u);
        if (d > 0) {
            feature.depth = d;
            feature.right = feature.keyPoint.pt.x - camera->basef / d;
        }
        feature.mapPoint = NULL;
    }
}

bool Features::getGridPos(const cv::KeyPoint& kp, int& posX, int& posY) {
        posX = round((kp.pt.x-camera->minX)*camera->gridElementWidthInv);
        posY = round((kp.pt.y-camera->minY)*camera->gridElementHeightInv);
        if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS) {
                return false;
        }
        return true;
}

void Features::assignFeaturesToGrid () {
        int reserve = 0.5f*keyPointsNum/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
        for (int i = 0; i < FRAME_GRID_COLS; i++) {
                for (int j = 0; j < FRAME_GRID_ROWS; j++) {
                        grid[i][j].reserve(reserve);
                }
        }
        for (int i = 0; i < keyPointsNum; i++) {
                const Feature& kp = keyPoints[i];
                int x, y;
                if (getGridPos (kp.keyPoint, x, y)) {
                        grid[x][y].push_back (i);
                }
        }
}

void Features::computeBow() {
    vector<cv::Mat> des;
    des.clear();
    for (int i = 0; i < rawKeyPoints.size(); i++) {
        des.push_back(descriptors.row(i));
    }
    voc->transform(des, bowVec, featVec, 4);
}

Features::Features(cv::Mat& image, const cv::Mat& depth) {
        //test(extractor, image);
        (*extractor)(image, cv::Mat(), rawKeyPoints, descriptors);
        //cout << rawKeyPoints.size() << endl;
        keyPointsNum = rawKeyPoints.size ();
        cout << "features keypointsnum " << keyPointsNum << endl;
        if (keyPointsNum == 0) {
                return;
        }
        keyPoints.resize(keyPointsNum);
        undistort ();
        getSteroMatch (depth);
        computeBow();
        assignFeaturesToGrid ();
}

vector<int> Features::getFeaturesInArea(const float& x, const float& y, const float& r, const int minLevel, const int maxLevel) {
    vector<int> result;
    result.clear();
    int cellX0, cellX1, cellY0, cellY1;
    if (!camera->getGridBound (x, y, r, cellX0, cellX1, cellY0, cellY1)) {
        return result;
    }
    for (int i = cellX0; i <= cellX1; i++) {
        for (int j = cellY0; j <= cellY1; j++) {
            const vector<int>& idx = grid[i][j];
            for (int k = 0; k < idx.size(); k++) {
                const cv::KeyPoint& kp = keyPoints[idx[k]].keyPoint;
                if (kp.octave < minLevel) {
                    continue;
                }
                if (maxLevel != -1 && kp.octave > maxLevel) {
                    continue;
                }
                float dx = kp.pt.x - x;
                float dy = kp.pt.y - y;
                if (fabs(dx) < r && fabs(dy) < r) {
                    result.push_back(idx[k]);
                }
            }
        }
    }
    return result;
}

void Features::eraseMapPoint(int index) {
    Feature& f = keyPoints[index];
    f.mapPoint = static_cast<MapPoint*>(NULL);
}

void Features::setMapPoint(int index, MapPoint *p) {
    Feature& f = keyPoints[index];
    f.mapPoint = p;
}

}
