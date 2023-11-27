#pragma once

#include <Eigen/Core>
#include <memory>

#include "PiecewiseFunction.h"
#include "piecewisefunctiondata.h"

namespace ofc::functionLib {

void createPiecewiseCubicInterpolation(PiecewiseFunction** traj, std::shared_ptr<PiecewiseFunctionData>& functionData, const Eigen::VectorXd& time, const Eigen::MatrixXd& pts);
void createPiecewiseCubicInterpolation(PiecewiseFunction** traj, std::shared_ptr<PiecewiseFunctionData>& functionData, const Eigen::VectorXd& time, const Eigen::MatrixXd& pts, const Eigen::MatrixXd& dpts);


void createPiecewiseLinearInterpolation(PiecewiseFunction** traj, std::shared_ptr<PiecewiseFunctionData>& functionData, const Eigen::VectorXd& time, const Eigen::MatrixXd& pts);

}
