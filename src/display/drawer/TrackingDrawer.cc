#include "TrackingDrawer.h"
#include "Frame.h"
#include "display.h"
#include "Converter.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace lqlslam {

TrackingDepthDrawer::TrackingDepthDrawer() {
    trackedPcl = new PointCloud::Ptr(new PointCloud);
}

PointCloud::Ptr TrackingDepthDrawer::singleFrameDepth(const cv::Mat&rgb, const cv::Mat& depth) {
    PointCloud::Ptr cloud(new PointCloud);
/*
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
*/
    PointT p;
    p.z = 0;
    p.x = 0;
    p.y = 0;
    p.r = 255;
    p.g = 0;
    p.b = 0;
    cloud->points.push_back(p);
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
