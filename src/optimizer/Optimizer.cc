#include "Optimizer.h"

namespace lqlslam {

TrackEdge* Optimizer::getTrackEdge(g2o::SparseOptimizer& optimizer, const Feature& feature) {
    const MapPoint* point = feature.mapPoint;
    const float delta = sqrt(5.991);
    if (point == NULL) {
        return NULL;
    }
    Eigen::Matrix<double, 2, 1> obs;
    const cv::KeyPoint& kp = feature.keyPoint;
    obs<< kp.pt.x , kp.pt.y;
    TrackEdge* e = new TrackEdge();
    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
    e->setMeasurement(obs);
    const float invSigma2 = extract->invLevelSigma2[kp.octave];
    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    rk->setDelta(delta);
    e->fx = camera->fx;
    e->fy = camera->fy;
    e->cx = camera->cx;
    e->cy = camera->cy;
    cv::Mat x = point->pos;
    e->Xw[0] = x.at<float>(0);
    e->Xw[1] = x.at<float>(1);
    e->Xw[2] = x.at<float>(2);
    return e;
}

int Optimizer::poseOptimize(Frame *frame) {
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::mat2SE3Quat(frame->pose.mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);
    Features* features = frame->features;
    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> edges;
    vector<int> edgeIndices;
    int num = features->keyPointsNum;
    int pointsNum = 0;
    for (int i = 0; i < num; i++) {
        Feature& feature = features->keyPoints[i];
        TrackEdge* e = Optimizer::getTrackEdge(optimizer, feature);
        if (e == NULL) {
            continue;
        }
        pointsNum ++;
        feature.isOutlier = false;
        optimizer.addEdge(e);
        edges.push_back(e);
        edgeIndices.push_back(i);
    }
    if (pointsNum < 3) {
        return 0;
    }
    cout << "origin points num: " << pointsNum << endl;
    int outlier = 0;
    for (int iter = 0; iter < 4; iter ++) {
        vSE3->setEstimate(Converter::mat2SE3Quat(frame->pose.mTcw));
        optimizer.initializeOptimization(0);
        optimizer.optimize(10);
        outlier = 0;
        for (int i = 0; i < edges.size(); i++) {
            TrackEdge* e = edges[i];
            int featureIdx = edgeIndices[i];
            Feature& feature = frame->features->keyPoints[featureIdx];
            if (feature.isOutlier) {
                e->computeError();
            }
            const float chi2 = e->chi2();
            //cout << "chi2:" << chi2 << endl;
            if (chi2 > 5.991) {
                feature.isOutlier = true;
                e->setLevel(1);
                outlier ++;
            } else {
                feature.isOutlier = false;
                e->setLevel(0);
            }
            if (iter == 2) {
                e->setRobustKernel(0);
            }
        }
    }
    for (int i = 0; i < frame->features->keyPointsNum; i++) {
        Feature& feature =frame->features->keyPoints[i];
        if (feature.isOutlier) {
            feature.mapPoint = NULL;
        }
    }
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    //todo set pose
    cv::Mat mat = Converter::toCvMat(SE3quat_recov);
    frame->setPose(mat);
    return pointsNum - outlier;
}

}
