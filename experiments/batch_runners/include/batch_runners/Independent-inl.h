#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <jrl/Dataset.h>
#include <jrl/Results.h>
#include <jrl/Types.h>

#include "batch_runners/Independent.h"

namespace batch_runners {
/*********************************************************************************************************************/
template <class POSE_TYPE>
BatchIterResults Independent<POSE_TYPE>::init() {
  std::vector<char> robots = this->dataset_.robots();
  jrl::Results result(this->dataset_.name(), this->name_, robots);
  for (auto& rid : robots) {
    result.robot_solutions[rid] = this->dataset_.initializationWithTypes(rid);
  }
  return BatchIterResults(result, 0);
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
BatchIterResults Independent<POSE_TYPE>::iterate() {
  // Setup the results
  std::vector<char> robots = this->dataset_.robots();
  jrl::Results result(this->dataset_.name(), this->name_, robots);

  // Remove any inter-robot measurement and then solve independently
  for (auto& rid : this->dataset_.robots()) {
    gtsam::NonlinearFactorGraph indep_fg;
    gtsam::Values indep_vals;

    // Remove any inter-robot factors
    for (auto& entry : this->dataset_.measurements(rid)) {
      for (auto& factor : entry.measurements) {
        bool include_factor = true;
        // Include the factor iff it contains only self referential keys
        for (auto& key : factor->keys()) {
          include_factor = include_factor && gtsam::Symbol(key).chr() == rid;
        }
        if (include_factor) {
          indep_fg.push_back(factor);
        }
      }
    }

    // Remove any variables from other robots
    jrl::TypedValues typed_initialization = this->dataset_.initializationWithTypes(rid);
    for (auto kvp : typed_initialization.values) {
      if (gtsam::Symbol(kvp.key).chr() == rid) {
        indep_vals.insert(kvp.key, kvp.value);
      }
    }

    // Optimize this robots factor graph independently
    gtsam::LevenbergMarquardtOptimizer optimizer(indep_fg, indep_vals);
    auto indep_solution = optimizer.optimize();
    result.robot_solutions[rid] = jrl::TypedValues(indep_solution, typed_initialization.types);
  }

  // Augment the results
  // We want to provide a solution that includes all variables that the non-independent robot considers
  for (auto& rid : this->dataset_.robots()) {
    for (auto& kvp : result.robot_solutions[rid].types) {
      char oid = gtsam::Symbol(kvp.first).chr();
      if (oid != rid) {
        result.robot_solutions[rid].values.insert(kvp.first, result.robot_solutions[oid].values.at(kvp.first));
      }
    }
  }

  return BatchIterResults(result, 0);
}
/*********************************************************************************************************************/
template <class POSE_TYPE>
bool Independent<POSE_TYPE>::isConverged() {
  return true;
}
}  // namespace batch_runners