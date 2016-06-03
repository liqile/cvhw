#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <list>
#include <opencv/cv.h>
#include "parameter.h"

using namespace std;

namespace lqlslam {

class Svd {

private:

    static CameraParam* getCameraParam();

    static cv::Mat getK();

public:

    static cv::Mat getFundamental(
            const vector<cv::KeyPoint>& kp1,
            const vector<cv::KeyPoint>& kp2,
            vector<cv::KeyPoint>& keyPoints1,
            vector<cv::KeyPoint>& keyPoints2
    );

    static cv::Mat getRt(
            const vector<cv::KeyPoint>& kp1,
            const vector<cv::KeyPoint>& kp2,
            const cv::Mat& F,
            vector<cv::Mat>& points
    );

};

}
