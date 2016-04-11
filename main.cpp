#include <iostream>
#include <string>
#include "ORBextractor.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace lqlslam;
int main()
{

    std::vector<cv::KeyPoint> keyPoints;
    cv::Mat descriptor;
    ORBextractor* extractor;
    int nFeatures = 1000;
    float fScaleFactor = 1.2;
    int nLevels = 8;
    int fIniThFAST = 20;
    int fMinThFAST = 7;
/*    ORBextractor.nFeatures: 1000

# ORB Extractor: Scale factor between levels in the scale pyramid
    ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid
    ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
# You can lower these values if your images have low contrast
    ORBextractor.iniThFAST: 20
    ORBextractor.minThFAST: 7
    */
    std::string imgFile = "/home/liqile/data/fr1_xyz/rgb/1305031102.175304.png";
    cv::Mat rgb = cv::imread (imgFile, CV_LOAD_IMAGE_UNCHANGED);
    if (rgb.channels () == 3)
        cvtColor(rgb, rgb, CV_RGB2GRAY);
    else if (rgb.channels () == 4) {
        cvtColor(rgb,rgb,CV_RGBA2GRAY);
    }
    extractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    (*extractor) (rgb, cv::Mat(), keyPoints, descriptor);
    cout <<" test extract: " << keyPoints.size () << endl;
    return 0;
}

