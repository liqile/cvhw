#ifndef DISPLAY_H
#define DISPLAY_H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace lqlslam {
class Display {
private:
    /*
     * image of matches
     */
    cv::Mat matchImg;

    /*
     * point cloud of tracking,
     * used only in "ONLY_TRACKING" mode
     */
    PointCloud::Ptr trackingPCL;

    /*
     * viewer of pcl in tracking
     */
    pcl::visualization::CloudViewer*  viewer;
public:
    Display();
    /*
     * setMatchGraph
     * @param cv::Mat img, the image of matches
     * function: set image of matches
     */
    void setMatchGraph(cv::Mat img);
    /*
     * setTrackingPCL
     * @param PointCloud::Ptr pcl, point cloud of tracking
     * function: set pcl of tracking
     */
    void setTrackingPCL(PointCloud::Ptr pcl);
    /*
     * show
     * function: display informations
     */
    void show();

    /*
     * start
     * function: start displayer
     */
    static void start();
};
extern Display* displayer;
}

#endif
