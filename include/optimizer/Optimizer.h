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
    int static poseOptimize(Frame* frame);
};

}

#endif
