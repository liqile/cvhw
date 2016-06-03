#include "display.h"

namespace lqlslam {

Display* displayer;

Display::Display() {
    viewer =new pcl::visualization::CloudViewer("tracking pcl");
}

void Display::setMatchGraph(cv::Mat img) {
    matchImg = img;
}

void Display::setTrackingMatchGraph(cv::Mat img) {
    trackingMatchImg = img;
}

void Display::setTrackingPCL(PointCloud::Ptr pcl){
    trackingPCL = pcl;
}

bool Display::show() {
    viewer->showCloud(trackingPCL);

    bool imgShow = false;
    if (!trackingMatchImg.empty()) {
        cv::imshow("tracking matches", trackingMatchImg);
        imgShow = true;
    }
    if (!matchImg.empty()) {
        cv::imshow("matches", matchImg);
        imgShow = true;
    }
    char ch = ' ';
    if (imgShow) {
        ch = cv::waitKey(30);
    }
    //if (ch != ' ') {
    //    return true;
    //}
    //cv::waitKey(30);
    return false;
}

void Display::start() {
    displayer = new Display();
}

}
