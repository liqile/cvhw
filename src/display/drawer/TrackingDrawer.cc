#include "TrackingDrawer.h"
#include "Frame.h"
#include "display.h"
#include "Converter.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace lqlslam {

void MatcherCounter::reset(const Features* curr, const Features* last) {
    lastIdx.clear();
    for (int i = 0; i < curr->keyPointsNum; i++) {
        lastIdx.push_back(-1);
    }
    currIdx.clear();
    for (int i = 0; i < last->keyPointsNum; i++) {
        currIdx.push_back(-1);
    }
    lastKeyPointNum = last->keyPointsNum;
    lastMapPointNum = 0;
    for (int i = 0; i < last->keyPointsNum; i++) {
        const Feature& feature = last->keyPoints[i];
        if (feature.mapPoint == NULL) {
            continue;
        }
        lastMapPointNum ++;
    }
    lastProjectNum = 0;
    lastMatchNum = 0;
    lastMatchFilterNum = 0;
}

void MatcherCounter::print() {
    cout << "[Matcher] last frame keypoints num: " << lastKeyPointNum << endl;
    cout << "[Matcher] last frame map point num: " << lastMapPointNum << endl;
    cout << "[Matcher] last frame map point reproject num: " << lastProjectNum << endl;
    cout << "[Matcher] last frame matches num: " << lastMatchNum << endl;
    cout << "[Matcher] last frame matches num (after filter): " << lastMatchFilterNum << endl;
}

void MatcherCounter::addMatch(int currIdx, int lastIdx) {
    this->lastIdx[currIdx] = lastIdx;
    this->currIdx[lastIdx] = currIdx;
    lastMatchNum ++;
    lastMatchFilterNum ++;
}

void MatcherCounter::reduceMatch(int currIdx) {
    lastIdx[currIdx] = -1;
    lastMatchFilterNum --;
}

void MatcherCounter::drawMatches(const Frame *curr, const Frame *last) {
    print();
    const vector<cv::KeyPoint>& kp2 = last->features->rawKeyPoints;
    const vector<cv::KeyPoint>& kp1 = curr->features->rawKeyPoints;
    const cv::Mat& img2 = last->rawData->img;
    const cv::Mat& img1 = curr->rawData->img;
    vector<cv::DMatch> matches;
    matches.clear();
    for (int i = 0; i < lastIdx.size(); i++) {
        if (lastIdx[i] == -1) {
            continue;
        }
        cv::DMatch match;
        match.queryIdx = lastIdx[i];
        match.trainIdx = i;
        matches.push_back(match);
    }
    cv::drawMatches(img2, kp2, img1, kp1, matches, match, cv::Scalar(255, 0, 0));
    displayer->setMatchGraph(match);
}

void MatcherCounter::drawTrackingMatches(const Frame *curr, const Frame *last) {
    const vector<cv::KeyPoint>& kp1 = curr->features->rawKeyPoints;
    const vector<cv::KeyPoint>& kp2 = last->features->rawKeyPoints;
    cv::Mat img1 = curr->rawData->img.clone();
    for (int i = 0; i < lastIdx.size(); i++) {
        if (lastIdx[i] == -1) {
            continue;
        }
        const cv::KeyPoint& k1 = kp1[i];
        const cv::KeyPoint& k2 = kp2[lastIdx[i]];
        cv::line(img1, k1.pt, k2.pt, cv::Scalar(255, 0, 0));
    }
    displayer->setTrackingMatchGraph(img1);
}

void MatcherCounter::addLastProjectNum() {
    lastProjectNum ++;
}

TrackingDepthDrawer::TrackingDepthDrawer() {
    trackedPcl = new PointCloud::Ptr(new PointCloud);
}

PointCloud::Ptr TrackingDepthDrawer::singleFrameDepth(const cv::Mat&rgb, const cv::Mat& depth) {
    PointCloud::Ptr cloud(new PointCloud);
    for (int m = 0; m < depth.rows; m++) {
        for (int n = 0; n < depth.cols; n++) {
            float d = depth.ptr<float>(m)[n];
            if (d <= 0) {
                continue;
            }
            PointT p;
            p.z = d;
            p.x = (n - camera->cx) * p.z / camera->fx;
            p.y = (m - camera->cy) * p.z / camera->fy;
            p.r = rgb.ptr<uchar>(m)[n * 3];
            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
            p.b = rgb.ptr<uchar>(m)[n * 3 + 2];
            cloud->points.push_back(p);
        }
    }
    return cloud;
}

void TrackingDepthDrawer::drawDepth(const cv::Mat& rgb, const cv::Mat& depth, const Pose& pose) {
    //[todo] : combine point clouds with old frames
    PointCloud::Ptr newPcl = singleFrameDepth(rgb, depth);
    for (int i = 0; i < newPcl->points.size(); i++) {
        PointT& p = newPcl->points[i];
        cv::Mat x3DC = (cv::Mat_<float>(3,1) << p.x, p.y, p.z);
        cv::Mat wp = pose.toWorld(x3DC);
        p.x = wp.at<float>(0, 0);
        p.y = wp.at<float>(1, 0);
        p.z = wp.at<float>(2, 0);
    }
    /*
    PointCloud::Ptr output(new PointCloud());
    Eigen::Isometry3d mTcw = Converter::toEigenT(pose.mTcw);
    cout << "eigen pose: " << mTcw.matrix() << endl;

    pcl::transformPointCloud(*newPcl, *output, mTcw.matrix());
    */
    **trackedPcl += *newPcl;
    static pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(0.02, 0.02, 0.02);
    voxel.setInputCloud(*trackedPcl);
    PointCloud::Ptr tmp(new PointCloud());
    voxel.filter(*tmp);
    *trackedPcl = tmp;
    PointCloud::Ptr localPcl(new PointCloud);
    for (int i = 0; i < (*trackedPcl)->points.size(); i++) {
        PointT p = (*trackedPcl)->points[i];
        cv::Mat x3DC = (cv::Mat_<float>(3,1) << p.x, p.y, p.z);
        cv::Mat lp = pose.toLocal(x3DC);
        p.x = lp.at<float>(0, 0);
        p.y = lp.at<float>(1, 0);
        p.z = lp.at<float>(2, 0);
        localPcl->points.push_back(p);
    }
    displayer->setTrackingPCL(localPcl);
}

}
