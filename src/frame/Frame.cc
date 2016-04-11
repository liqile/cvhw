#include "Frame.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace std;

namespace lqlslam {

void testFrameFeature(ORBextractor* extractor, const cv::Mat& image) {
    std::vector<cv::KeyPoint> keyPoints;
    cv::Mat descriptor;
    (*extractor) (image, cv::Mat(), keyPoints, descriptor);
    cout <<" test frame extract: " << keyPoints.size () << endl;
}

RawData::RawData(const cv::Mat& image, const cv::Mat& depthInfo, const double& timestamp) {
    img = image.clone();
    depth = depthInfo.clone();
    this->timeStamp = timestamp;
}

int Frame::nextId = 0;

int Frame::nextKFId = 0;

void Frame::generate3dPoints() {
#if ONLY_TRACKING
    for (int i = 0; i < features->keyPointsNum; i++) {
        cv::Mat p;
        Feature& feature = features->keyPoints[i];
        if (feature.mapPoint != NULL) {
            delete feature.mapPoint;
            feature.mapPoint = NULL;
        }
        if (feature.unProject (pose, p)) {
            feature.mapPoint = new MapPoint(p, this, i);
        } else {
            feature.mapPoint = NULL;
        }
    }
#endif
}

Frame::Frame(
        const cv::Mat& imGray,
        const cv::Mat& depth,
        const double& timeStamp
) {
        //testFrameFeature (extractor, imGray);
        //exit(0);
        frameId = nextId ++;
        isKeyFrame = false;
        rawData = new RawData(imGray, depth, timeStamp);
        features = new Features(rawData->img, depth);
}

void Frame::setPose(const cv::Mat& mTcw) {
    pose.setPose (mTcw);
}

bool Frame::unProject(const int& i, cv::Mat& p) {
    Feature& f = features->keyPoints[i];
    return f.unProject (pose, p);
}

void Frame::becomeKeyframe () {
    isKeyFrame = true;
    keyFrameId = nextKFId;
    nextKFId ++;
    generate3dPoints();
}
#if 0
void Frame::reset() {
    setPose (cv::Mat::eye(4,4,CV_32F));
    for (int i = 0; i < features->keyPointsNum; i++) {
        cv::Mat p;
        Feature& feature = features->keyPoints[i];
        if (feature.unProject (pose, p)) {
            feature.mapPoint = new MapPoint(p);
        } else {
            feature.mapPoint = NULL;
        }
    }
}
#endif
void testFrame() {
    Param* param = readParam ("/home/liqile/qtwork/lqlslam/param.txt");
    CameraParam* camera = param->getCamera ();
    ExtractParam extract = *(param->getExtract ());
    std::string depthFile = "/home/liqile/data/fr1_xyz/depth/1305031102.160407.png";
    std::string imgFile = "/home/liqile/data/fr1_xyz/rgb/1305031102.175304.png";
    cv::Mat rgb = cv::imread (imgFile);
    cvtColor(rgb, rgb, CV_RGB2GRAY);
    cv::imshow ("display", rgb);
    cv::waitKey ();
    cv::Mat depth = cv::imread(depthFile);
    imshow("depthraw", depth);
    depth.convertTo (depth, CV_32F, 5000.0);
    cv::imshow("depth", depth);
    cv::waitKey();
    Frame* frame = new Frame(rgb, depth, 0);
    std::vector<cv::Mat>& v = extractor->mvImagePyramid;
    //for (int i = 0; i < v.size (); i++) {
    //    cv::imshow ("aa", extractor->mvImagePyramid[i]);
    //    cv::waitKey ();
    //}
    cout << frame->features->keyPointsNum << endl;
}

}
