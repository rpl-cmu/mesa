#pragma once
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <jrl/Types.h>

#include "batch_runners/Centralized.h"

namespace batch_runners {
/*********************************************************************************************************************/
template <class POSE_TYPE>
BatchIterResults Centralized<POSE_TYPE>::init() {
  // Aggregate all the measurements and initialization into a single factor graph
  for (auto& rid : this->dataset_.robots()) {
    for (auto& entry : this->dataset_.measurements(rid)) {
      joint_factor_graph_.push_back(entry.measurements);
    }

    for (auto kvp : this->dataset_.initialization(rid)) {
      if (!joint_values_.exists(kvp.key)) {
        joint_values_.insert(kvp.key, kvp.value);
      }
    }
  }

  std::vector<char> robots = this->dataset_.robots();
  jrl::Results result(this->dataset_.name(), this->name_, robots);
  for (auto& rid : robots) {
    result.robot_solutions[rid] = this->dataset_.initializationWithTypes(rid);
  }
  return BatchIterResults(result, 0);
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
BatchIterResults Centralized<POSE_TYPE>::iterate() {
  // Solve
  gtsam::LevenbergMarquardtOptimizer optimizer(joint_factor_graph_, joint_values_);
  auto joint_solution = optimizer.optimize();

  // Aggregate the results
  std::vector<char> robots = this->dataset_.robots();
  jrl::Results result(this->dataset_.name(), this->name_, robots);
  for (auto& rid : robots) {
    gtsam::Values values;
    jrl::TypedValues initialization = this->dataset_.initializationWithTypes(rid);
    for (auto kvp : initialization.values) {
      values.insert(kvp.key, joint_solution.at(kvp.key));
    }
    result.robot_solutions[rid] = jrl::TypedValues(values, initialization.types);
  }
  return BatchIterResults(result, 0);
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
bool Centralized<POSE_TYPE>::isConverged() {
  return true;
}

}  // namespace batch_runners