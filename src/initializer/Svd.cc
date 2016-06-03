#include "Svd.h"

namespace lqlslam {

CameraParam* Svd::getCameraParam() {
    return camera;
}

cv::Mat Svd::getK() {
    CameraParam* c = getCameraParam();
    return c->k;
}

}
