#ifndef PARAMETER_H
#define PARAMETER_H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "ORBextractor.h"

using namespace std;

namespace lqlslam {

#define DEBUG_MATCHER 1
#define ONLY_TRACKING 0
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

struct CameraParam {
    int rows;
    int cols;
    cv::Mat k;
    cv::Mat distCoef;
    float fx;
    float fy;
    float cx;
    float cy;
    float invFx;
    float invFy;
    float base;
    float basef;
    float minX;
    float minY;
    float maxX;
    float maxY;
    float gridElementWidthInv;
    float gridElementHeightInv;
    void undistortBound();
    CameraParam(const cv::Mat& k, const cv::Mat& distCoef, float basef, int rows, int cols);
    bool getGridBound(const float &x, const float &y, const float &r, int& cellX0, int& cellX1, int& cellY0, int& cellY1);
    /*
     * project
     * @param const cv::Mat& localPos, pos of point in camera system
     * @param float& x, store projection coordinate
     * @param float& y, store projection coordinate
     * function: project local pos to frame, compute x, y
     */
    void project(const cv::Mat& localPos, float& x, float& y);
    //bool unProject(const cv::Mat& p, float& u, float& v, float& invz, float& ur);
};

struct ExtractParam {
    int nFeatures;
    float fScaleFactor;
    int nLevels;
    int fIniThFAST;
    int fMinThFAST;
    vector<float> scaleFactor;
    vector<float> levelSigma2;
    vector <float> invLevelSigma2;
};

struct Setting {
    string vocFile;
    string dataRoot;
};

struct Param {
    CameraParam* camera;
    ExtractParam* extract;
    Setting* setting;
    Param(CameraParam* camera, ExtractParam* extract, Setting* setting) {
        this->camera = camera;
        this->extract = extract;
        this->setting = setting;
    }
    CameraParam* getCamera() {
        return camera;
    }
    ExtractParam* getExtract() {
        return extract;
    }
};

class ParamReader {
    public:
    ParamReader(const string& fileName);
    map<string, float> data;
    map<string, string> strs;
};

extern CameraParam* camera;
extern ExtractParam* extract;
extern Setting* setting;
extern Param* param;
extern ORBextractor* extractor;

Param* readParam(const std::string& fileName);

void setParam(const std::string& fileName);

void testParam();

}

#endif
