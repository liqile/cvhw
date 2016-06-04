#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <list>
#include <opencv/cv.h>
#include "parameter.h"
#include "Pose.h"

using namespace std;

namespace lqlslam {

class Svd {

private:

    static CameraParam* getCameraParam();

    static cv::Mat getK();

    static void getAllPoses(const cv::Mat& F, cv::Mat& R1, cv::Mat& R2, cv::Mat& t1, cv::Mat& t2);

    static void getDepth(const Pose& p2, const vector<cv::KeyPoint> &kp1, const vector<cv::KeyPoint> &kp2, vector<cv::Mat>& points);

    static int countPositive(const Pose& pose, const vector<cv::Mat>& points);

public:

    static cv::Mat getFundamental(
            const vector<cv::Point2f>& p1,
            const vector<cv::Point2f>& p2,
            vector<bool>& mask
    );

    static bool getRt(
            const vector<cv::KeyPoint>& kp1,
            const vector<cv::KeyPoint>& kp2,
            const cv::Mat& F,
            Pose& pose
    );

};

}
