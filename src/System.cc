#include "System.h"
#include "Tracking.h"
#include <fstream>
#include "display.h"
#include "drawers.h"
#include "ORBVocabulary.h"

using namespace std;

namespace lqlslam {

void System::initialize (Frame* frame) {
    //frame->setPose(cv::Mat::eye(4,4,CV_32F));
    tracker->initialize(frame);
    //KeyFrame* keyFrame = new KeyFrame(frame);
    //keyFrame->reset();
    //map->clear ();
    //map->addKeyFrame(keyFrame);
}

System::System(const string& fileName) {
    setParam (fileName);
    cout << "nFeatures:" << extract->nFeatures << endl;
    cout << "fScaleFactor:" << extract->fScaleFactor << endl;
    cout << "nLevels:" << extract->nLevels << endl;
    cout << "fIniTh:" << extract->fIniThFAST << endl;
    cout << "fMin:" << extract->fMinThFAST << endl;
    cout << "vocFile: " << setting->vocFile << endl;
    loadVocabulary(setting->vocFile);
    state = 0;
    map = new Map();
    tracker = new Tracking();
    Display::start();
}

Pose System::track(const cv::Mat& img, const cv::Mat& depth, const double& timestamp) {
    //cv::imshow("img", img);
    //cv::waitKey(0);
    Frame* frame = new Frame(img, depth, timestamp);
    cout << "keyPoints num " << frame->features->keyPointsNum << endl;
    if (state == 0) {
        initialize(frame);
        state = 1;
    } else {
        int points = tracker->trackLastFrame(frame);
        cout << "track points: " << points << endl;
        cout << "pose: " << frame->pose.mTcw << endl;
    }
    return frame->pose;
}

void System::test(const cv::Mat& image) {
    std::vector<cv::KeyPoint> keyPoints;
    cv::Mat descriptor;
    (*extractor) (image, cv::Mat(), keyPoints, descriptor);
    cout <<" test extract: " << keyPoints.size () << endl;
}

void readImage(const string& imgName, const string& depthName, cv::Mat& img, cv::Mat& depth, cv::Mat& rgb) {
    std::string depthFile = setting->dataRoot + "/depth/" + depthName;
    std::string imgFile = setting->dataRoot + "/rgb/" + imgName;
    cout << "=================="<<endl;
    cout <<depthName<<endl;
    cout <<imgName<<endl;
    cout << depthFile << endl;
    cout << imgFile << endl;
    cv::Mat rgb_tmp = cv::imread (imgFile, CV_LOAD_IMAGE_UNCHANGED);
    rgb = rgb_tmp.clone();
    //cv::imshow("readImage", rgb);
    if (rgb_tmp.channels () == 3)
    cvtColor(rgb_tmp, rgb_tmp, CV_RGB2GRAY);
    else if (rgb_tmp.channels () == 4) {
        cvtColor(rgb_tmp,rgb_tmp,CV_RGBA2GRAY);
    }
    img = rgb_tmp;
    depth = cv::imread(depthFile, CV_LOAD_IMAGE_UNCHANGED);
    depth.convertTo (depth, CV_32F, 1.0 / 5000.0);
}

void testSystem3() {
    System* system = new System("../param.txt");
    cout << setting->dataRoot<< endl;
    vector<string> rgb;
    readFileNames(setting->dataRoot + "/rgb.txt", rgb);
    vector<string> depth;
    readFileNames(setting->dataRoot + "/depth.txt", depth);
    for (int i = 0; i < 1; i++) {
        cout << rgb[i] << " " << depth[i] << endl;
    }
    for (int i = 0; i < depth.size(); i++) {
        cv::Mat img;
        cv::Mat d;
        cv::Mat rgbImg;
        readImage(rgb[i], depth[i], img, d, rgbImg);
        Pose pose = system->track(img, d, 0);
        trackingDepthDrawer.drawDepth(rgbImg, d, pose);
        displayer->show();
    }
}

void readFileNames(const string& path, vector<string>& names) {
    cout << path << endl;
    ifstream fin(path);
    char s[256];
    while (!fin.eof()) {
        fin.getline (s, 256);
        int len = strlen(s);
        int i;
        for (i = 0; i < len; i++) {
            if (s[i] == '/') {
                break;
            }
        }
        string ss = &s[i + 1];
        //cout << ss << endl;
        names.push_back(ss);
    }
    names.pop_back();
    fin.close ();
}

}
