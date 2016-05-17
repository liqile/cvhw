#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Frame.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>
#include"Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include"Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Converter.h"
namespace lqlslam {

typedef g2o::EdgeSE3ProjectXYZOnlyPose TrackEdge;

class Optimizer {
private:
    static TrackEdge* getTrackEdge(g2o::SparseOptimizer& optimizer, const Feature& feature);
public:
    /*
     * poseOptimize
     * @param Frame* frame, current frame to estimate
     * function: estimate and set pose of frame.
     *     return matched mappoints num for tracking
     */
    static int poseOptimize(Frame* frame);
    /*
     * localBA
     * @param list<Frame*>& lf, frames to optimize in local map
     * @param list<MapPoint*>& lp, mappoints to optimze in local map
     * @param list<Frame*>& ff, fixed frames during optimization in local map
     * function: optimize local map by bundle adjustment
     *     frames in lf, mappoints in lp will be optimized
     */
    static void localBA(list<Frame*>& lf, list<MapPoint*>& lp, list<Frame*>& ff);
};

}

#endif
