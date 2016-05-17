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
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    //todo set pose
    cv::Mat mat = Converter::toCvMat(SE3quat_recov);
    frame->setPose(mat);
    return pointsNum - outlier;
}

void Optimizer::localBA(list<Frame *> &lf, list<MapPoint *> &lp, list<Frame *> &ff) {
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType* linerSolver;
    linerSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linerSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    list<Frame*>::iterator it = lf.begin();
    int maxFrameId = 0;
    while (it != lf.end()) {
        Frame* f = *it;
        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        cv::Mat mTcw = f->pose.mTcw;
        v->setEstimate(Converter::mat2SE3Quat(mTcw));
        v->setId(f->keyFrameId);
        v->setFixed(f->keyFrameId == 0);
        if (f->keyFrameId > maxFrameId) {
            maxFrameId = f->keyFrameId;
        }
        optimizer.addVertex(v);
        cout << "[local map] local frame: " << f->frameId << endl;
        it ++;
    }
    it = ff.begin();
    while (it != ff.end()) {
        Frame* f = *it;
        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        cv::Mat mTcw = f->pose.mTcw;
        v->setEstimate(Converter::mat2SE3Quat(mTcw));
        v->setId(f->keyFrameId);
        v->setFixed(true);
        if (f->keyFrameId > maxFrameId) {
            maxFrameId = f->keyFrameId;
        }
        optimizer.addVertex(v);
        it ++;
    }
    cout << "[local map] local ba , max key frame id: " << maxFrameId << endl;
    vector<g2o::EdgeSE3ProjectXYZ*> edges;
    edges.clear();
    vector<Frame*> fs;
    fs.clear();
    vector<MapPoint*> ps;
    ps.clear();
    list<MapPoint*>::iterator itp = lp.begin();
    int pointNum = 0;
    while (itp != lp.end()) {
        MapPoint* p = *itp;
        pointNum ++;
        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
        Eigen::Matrix<double, 3, 1> pos;
        pos << p->pos.at<float>(0), p->pos.at<float>(1), p->pos.at<float>(2);
        v->setEstimate(pos);
        int pid = p->pointId + maxFrameId + 1;
        v->setId(pid);
        v->setMarginalized(true);
        optimizer.addVertex(v);
        const map<Frame*, int>& obs = p->observation.observations;
        map<Frame*, int>::const_iterator fit = obs.begin();
        while (fit != obs.end()) {
            Frame* f = fit->first;
            const Feature& fe = f->features->keyPoints[fit->second];
            Eigen::Matrix<double, 2, 1> obs;
            obs << fe.keyPoint.pt.x, fe.keyPoint.pt.y;
            g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pid)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(f->keyFrameId)));
            e->setMeasurement(obs);
            float invSigma2 = extract->invLevelSigma2[fe.keyPoint.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber();
            e->setRobustKernel(rk);
            rk->setDelta(sqrt(5.991));
            e->fx = camera->fx;
            e->fy = camera->fy;
            e->cx = camera->cx;
            e->cy = camera->cy;
            optimizer.addEdge(e);
            edges.push_back(e);
            fs.push_back(f);
            ps.push_back(p);
            fit ++;
        }
        itp ++;
    }
    cout << "[local map] local ba, point num: " << pointNum << endl;
    //optmize
    optimizer.initializeOptimization();
    optimizer.optimize(5);
    for (int i = 0; i < edges.size(); i++) {
        g2o::EdgeSE3ProjectXYZ* e = edges[i];
        if (e->chi2() > 5.991 || !e->isDepthPositive()) {
            e->setLevel(1);
        }
        e->setRobustKernel(0);
    }
    optimizer.initializeOptimization(0);
    optimizer.optimize(10);
    for (int i = 0; i < edges.size(); i++) {
        g2o::EdgeSE3ProjectXYZ* e = edges[i];
        if (e->chi2() > 5.991 || !e->isDepthPositive()) {
            Frame* f = fs[i];
            MapPoint* p = ps[i];
            p->decObservation(f);
        }
    }
    it = lf.begin();
    while (it != lf.end()) {
        Frame* f = *it;
        g2o::VertexSE3Expmap* v = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(f->keyFrameId));
        g2o::SE3Quat pose = v->estimate();
        f->pose.setPose(Converter::toCvMat(pose));
        it ++;
    }
    itp = lp.begin();
    while (itp != lp.end()) {
        MapPoint* p = *itp;
        g2o::VertexSBAPointXYZ* v = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(p->pointId + maxFrameId + 1));
        p->setPointPos(Converter::toCvMat(v->estimate()));
        itp ++;
    }
}

}
