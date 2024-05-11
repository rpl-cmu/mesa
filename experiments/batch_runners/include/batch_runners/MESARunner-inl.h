#pragma once
#include <jrl/Metrics.h>

#include "batch_runners/MESARunner.h"

namespace batch_runners {
/***************************************************************************/
template <class POSE_TYPE, class MESA_METHOD>
BatchIterResults BatchMESARunner<POSE_TYPE, MESA_METHOD>::init() {
  // Accumulate data for solver format
  std::map<char, gtsam::NonlinearFactorGraph> robot_graphs;
  std::map<char, gtsam::Values> robot_estimates;
  std::vector<char> robots = this->dataset_.robots();
  for (auto& rid : robots) {
    robot_graphs[rid] = gtsam::NonlinearFactorGraph();
    robot_estimates[rid] = this->dataset_.initialization(rid);

    // Accumulate all measurements
    for (auto& entry : this->dataset_.measurements(rid)) {
      robot_graphs[rid].push_back(entry.measurements);
    }
  }

  // Construct and initialize the solver
  solver_ = std::make_shared<MESA_METHOD>(robots, robot_graphs, robot_estimates, params_);
  std::map<char, gtsam::Values> robot_init_results = solver_->init();

  jrl::Results result(this->dataset_.name(), this->name_, robots);
  for (auto& rid : robots) {
    jrl::TypedValues initialization = this->dataset_.initializationWithTypes(rid);
    result.robot_solutions[rid] = jrl::TypedValues(robot_init_results[rid], initialization.types);
  }

  return BatchIterResults(result, 0);
}

/***************************************************************************/
template <class POSE_TYPE, class MESA_METHOD>
BatchIterResults BatchMESARunner<POSE_TYPE, MESA_METHOD>::iterate() {
  // iterate the solver
  std::map<char, gtsam::Values> robot_results = solver_->iterate();
  // Aggregate the results
  std::vector<char> robots = this->dataset_.robots();
  jrl::Results result(this->dataset_.name(), this->name_, robots);
  for (auto& rid : this->dataset_.robots()) {
    jrl::TypedValues initialization = this->dataset_.initializationWithTypes(rid);
    result.robot_solutions[rid] = jrl::TypedValues(robot_results[rid], initialization.types);
  }

  return BatchIterResults(result, 1);
}

/***************************************************************************/
template <class POSE_TYPE, class MESA_METHOD>
bool BatchMESARunner<POSE_TYPE, MESA_METHOD>::isConverged() {
  return solver_->isConverged();
}

/***************************************************************************/
template <class POSE_TYPE, class MESA_METHOD>
size_t BatchMESARunner<POSE_TYPE, MESA_METHOD>::iterationsPerFullComms() {
  return solver_->commNetworkSize();
}

}  // namespace batch_runners