#include "parameter.h"
#include <math.h>
#include <fstream>
#include <string>
using namespace std;
namespace lqlslam{

CameraParam* camera;
ExtractParam* extract;
Setting* setting;
Param* param;
ORBextractor* extractor;

CameraParam::CameraParam(const cv::Mat& k, const cv::Mat& distCoef, float basef,  int rows, int cols) {
        this->rows = rows;
        this->cols = cols;
        this->k = k.clone ();
        this->distCoef = distCoef.clone ();
        fx = k.at<float>(0,0);
        fy = k.at<float>(1,1);
        cx = k.at<float>(0,2);
        cy = k.at<float>(1,2);
        invFx = 1.0f/fx;
        invFy = 1.0f/fy;
        this->basef = basef;
        this->base = this->basef / fx;
        undistortBound();
}

void CameraParam::undistortBound() {
    if (distCoef.at<float>(0)!=0.0) {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=rows;
        mat.at<float>(3,0)=cols; mat.at<float>(3,1)=rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,k,distCoef,cv::Mat(),k);
        mat=mat.reshape(1);

        minX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        maxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        minY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        maxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    } else {
        minX = 0.0f;
        maxX = cols;
        minY = 0.0f;
        maxY = rows;
    }
    gridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(maxX - minX);
    gridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(maxY - minY);
}

bool CameraParam::getGridBound (const float &x, const float &y, const float &r, int& cellX0, int& cellX1, int& cellY0, int& cellY1) {
    cellX0 = max(0,(int)floor((x-minX-r)*gridElementWidthInv));
    if(cellX0>=FRAME_GRID_COLS) {
        return false;
    }
    cellX1 = min((int)FRAME_GRID_COLS-1,(int)ceil((x-minX+r)*gridElementWidthInv));
    if(cellX1<0) {
        return false;
    }
    cellY0 = max(0,(int)floor((y-minY-r)*gridElementHeightInv));
    if(cellY0>=FRAME_GRID_ROWS) {
        return false;
    }
    cellY1 = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-minY+r)*gridElementHeightInv));
    if(cellY1<0) {
        return false;
    }
    return true;
}
ParamReader::ParamReader(const string& fileName) {
    data.clear();
    ifstream fin(fileName.c_str());
    while (!fin.eof()) {
        string str;
        getline(fin, str);
        if (str[0] == '#') {
            continue;
        }
        int pos = str.find(":");
        if (pos == -1) {
            continue;
        }
        string key = str.substr(0, pos);
        string value = str.substr(pos + 1, str.length());
//        if (key != "DataRoot" && key) {
            while (value[0] == ' ') {
                value = value.substr (1, value.length());
            }
            data[key] = atof(value.c_str());
            strs[key] = value;
//        } else {
//            while (value[0] == ' ') {
//                value = value.substr (1, value.length());
//            }
//            strs[key] = value;
//        }
        if (!fin.good()) {
            break;
        }
    }
}

Param* readParam(const std::string& fileName) {
    ParamReader reader(fileName);
    map<string, float>& fSettings = reader.data;
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    cv::Mat k = cv::Mat::eye(3,3,CV_32F);
    k.at<float>(0,0) = fx;
    k.at<float>(1,1) = fy;
    k.at<float>(0,2) = cx;
    k.at<float>(1,2) = cy;
    cv::Mat distCoef(4,1,CV_32F);
    distCoef.at<float>(0) = fSettings["Camera.k1"];
    distCoef.at<float>(1) = fSettings["Camera.k2"];
    distCoef.at<float>(2) = fSettings["Camera.p1"];
    distCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0) {
        distCoef.resize(5);
        distCoef.at<float>(4) = k3;
    }
    float basef = fSettings["Camera.bf"];
    int cols = fSettings["Camera.width"];
    int rows = fSettings["Camera.height"];
    CameraParam* camera = new CameraParam(k, distCoef, basef, rows, cols);
    ExtractParam* extract = new ExtractParam();
    extract->nFeatures = fSettings["ORBextractor.nFeatures"];
    extract->fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    extract->nLevels = fSettings["ORBextractor.nLevels"];
    extract->fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    extract->fMinThFAST = fSettings["ORBextractor.minThFAST"];
    extractor = new ORBextractor(
                extract->nFeatures,
                extract->fScaleFactor,
                extract->nLevels,
                extract->fIniThFAST,
                extract->fMinThFAST
    );
    extract->scaleFactor = extractor->GetScaleFactors ();
    extract->invLevelSigma2 = extractor->GetInverseScaleSigmaSquares();
    Setting* setting = new Setting();
    setting->dataRoot = reader.strs["DataRoot"];
    setting->vocFile = reader.strs["VocFile"];
    //setting->dataRoot = "/home/lql/slam/data/rgbd_dataset_freiburg1_xyz";
    return new Param(camera, extract, setting);
}

void setParam(const std::string& fileName) {
    param = readParam (fileName);
    camera = param->camera;
    extract = param->extract;
    setting = param->setting;
}

void testParam() {
    lqlslam::Param* param = lqlslam::readParam ("/home/liqile/qtwork/lqlslam/param.txt");
    lqlslam::CameraParam* camera = param->getCamera ();
    cout <<"**************camera****************"<<endl;
    cout <<camera->rows << endl;
    cout << camera->cols << endl;
    cout << camera->k << endl;
    cout << camera->distCoef << endl;
    cout << camera->fx << endl;
    cout << camera->fy << endl;
    cout << camera->cx << endl;
    cout << camera->cy << endl;
    cout << camera->invFx << endl;
    cout << camera->invFy << endl;
    cout << camera->base << endl;
    cout << camera->basef << endl;
    cout <<"undistort boundary"<<endl;
    cout << camera->minX << endl;
    cout << camera->minY << endl;
    cout << camera->maxX << endl;
    cout << camera->maxY << endl;
    cout << camera->gridElementWidthInv << endl;
    cout << camera->gridElementHeightInv << endl;
    cout <<"******************extract***************"<<endl;
    lqlslam::ExtractParam* extract = param->getExtract ();
    cout <<extract->nFeatures<<endl;
    cout << extract->fScaleFactor << endl;
    cout <<extract->nLevels<<endl;
    cout <<extract->fIniThFAST<<endl;
    cout  <<extract->fMinThFAST<<endl;
}

}

