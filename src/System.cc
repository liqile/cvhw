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
    frame->generate3dPoints();
    lastKeyFrame = frame;
    frame->becomeKeyframe();
    //map->addKeyFrame(lastKeyFrame);
    map->initialize(lastKeyFrame);
}
bool System::needKeyFrame(Frame *frame, int points) {
    //if (frame->frameId <= 1) {
    //    return true;
    //}

    Frame* ref = tracker->neigh[0];
    int num = 0;
    int tot = 0;
    for (int i = 0; i < ref->features->keyPointsNum; i++) {
        const Feature& f = ref->features->keyPoints[i];
        if (f.mapPoint != NULL) {
            tot ++;
            if (f.mapPoint->good) {
                num ++;
            }
        }
    }
    float rate = (float)points / num;
    cout << "[tracking] ref keyframe frameid:" << ref->frameId << endl;
    cout << "[tracking] ref keyframe map point: " << tot << endl;
    cout << "[tracking] ref keyframe map point (after culling): " << num << endl;
    cout << "[tracking] points tracking number: " << points << endl;
    cout << "[tracking] points tracking rate: " << rate << endl;
    int delta = frame->frameId - lastKeyFrame->frameId;
    if (delta >= 20 && rate < 0.9 || delta >= 8 && (rate < 0.6 || points <=35) || rate < 0.3) {
        return true;
    }
    //if (points < 90) {
    //    return true;
    //}
    return false;
}
void System::createNewKeyFrame(Frame* frame) {
    frame->becomeKeyframe();
    lastKeyFrame = frame;
    map->addKeyFrame(frame);
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
    map = new LocalMap();
    tracker = new Tracking();
    initializer = new Initializer(tracker, map);
    Display::start();
}

Pose System::track(const cv::Mat& img, const cv::Mat& depth, const double& timestamp) {
    //cv::imshow("img", img);
    //cv::waitKey(0);
    Frame* frame = new Frame(img, depth, timestamp);
    cout << "keyPoints num " << frame->features->keyPointsNum << endl;
    if (initializer->needInitialize()) {
        initializer->addFrame(frame);
        lastKeyFrame = initializer->getLastFrame();
    } else {
        int points = tracker->track(frame);
        cout << "track points: " << points << endl;
        cout << "pose: " << frame->pose.mTcw << endl;
        /*
        if (frame->frameId == 2) {
            lastKeyFrame = frame;
        } else {
            if (frame->frameId - lastKeyFrame->frameId > 3) {
                ORBmatcher matcher(frame);
                matcher.searchByTriangular(lastKeyFrame);
            }
        }
        */
        if (needKeyFrame(frame, points)) {
            createNewKeyFrame(frame);
        }
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
        cv::Mat dep;
        if (i == 0) {
            dep = d;
        }
        Pose pose = system->track(img, dep, 0);
        cout << " drawing depth of frame: " << i << endl;
        trackingDepthDrawer.drawDepth(rgbImg, d, pose);
        displayer->show();
        cout << " depth of frame: " << i << endl;
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
