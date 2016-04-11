#include "display.h"

namespace lqlslam {

Display* displayer;

Display::Display() {
    viewer =new pcl::visualization::CloudViewer("tracking pcl");
}

void Display::setMatchGraph(cv::Mat img) {
    matchImg = img;
}

void Display::setTrackingPCL(PointCloud::Ptr pcl){
    trackingPCL = pcl;
}

void Display::show() {
    viewer->showCloud(trackingPCL);
    if (!matchImg.empty()) {
        cv::imshow("matches", matchImg);
        cv::waitKey(0);
    }
}

void Display::start() {
    displayer = new Display();
}

}
