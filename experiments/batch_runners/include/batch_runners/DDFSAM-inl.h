
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <jrl/Dataset.h>
#include <jrl/Results.h>
#include <jrl/Types.h>

#include "batch_runners/DDFSAM.h"

namespace batch_runners {

/*********************************************************************************************************************/
template <class POSE_TYPE>
BatchIterResults DDFSAM<POSE_TYPE>::init() {
  std::vector<char> robots = this->dataset_.robots();
  jrl::Results result(this->dataset_.name(), this->name_, robots);
  for (auto& rid : robots) {
    result.robot_solutions[rid] = this->dataset_.initializationWithTypes(rid);
  }
  return BatchIterResults(result, 0);
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
BatchIterResults DDFSAM<POSE_TYPE>::iterate() {
  std::map<char, gtsam::NonlinearFactorGraph> robot_factor_graphs;
  std::map<char, gtsam::KeySet> robot_outside_variables;
  std::map<char, gtsam::Values> robot_values;
  std::map<char, gtsam::Marginals> robot_marginals;

  // Populate the robot graphs and values
  for (auto& rid : this->dataset_.robots()) {
    robot_factor_graphs[rid] = gtsam::NonlinearFactorGraph();
    robot_outside_variables[rid] = gtsam::KeySet();
    robot_values[rid] = this->dataset_.initialization(rid);
    for (auto& entry : this->dataset_.measurements(rid)) {
      for (auto& factor : entry.measurements) {
        // Accumulate the Factors
        robot_factor_graphs[rid].push_back(factor);
        // Accumulate any outside keys
        for (auto& key : factor->keys()) {
          char r = gtsam::Symbol(key).chr();
          if (r != rid) {
            robot_outside_variables[rid].insert(key);
          }
        }
      }
    }
  }

  // Optimize each robot independently
  std::map<char, gtsam::NonlinearFactorGraph> robot_independent_factor_graphs;
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
    robot_independent_factor_graphs[rid] = indep_fg;

    // Remove any variables from other robots
    jrl::TypedValues typed_initialization = this->dataset_.initializationWithTypes(rid);
    for (auto kvp : typed_initialization.values) {
      if (gtsam::Symbol(kvp.key).chr() == rid) {
        indep_vals.insert(kvp.key, kvp.value);
      }
    }

    // Optimize the Independent Graph
    gtsam::LevenbergMarquardtOptimizer optimizer(indep_fg, indep_vals);
    gtsam::Values indep_solution = optimizer.optimize();

    // Compute the Marginals
    robot_marginals[rid] = gtsam::Marginals(indep_fg, indep_solution);

    // Update the initialization with the independent solution
    robot_values[rid].update(indep_solution);
  }

  // Add Summarized Graphs and Optimize
  std::map<char, gtsam::Values> optimized_values;
  for (auto& rid : this->dataset_.robots()) {
    gtsam::NonlinearFactorGraph augmented_fg;
    augmented_fg.push_back(robot_factor_graphs[rid]);

    // Add priors for each shared variable
    for (auto& ok : robot_outside_variables[rid]) {
      char r = gtsam::Symbol(ok).chr();
      augmented_fg.push_back(gtsam::PriorFactor<POSE_TYPE>(
          ok, robot_values[r].at<POSE_TYPE>(ok),
          gtsam::noiseModel::Gaussian::Covariance(robot_marginals[r].marginalCovariance(ok))));
      robot_values[rid].update(ok, robot_values[r].at<POSE_TYPE>(ok));
    }
    gtsam::LevenbergMarquardtOptimizer optimizer(augmented_fg, robot_values[rid]);
    optimized_values[rid] = optimizer.optimize();
  }

  // Aggregate the results
  std::vector<char> robots = this->dataset_.robots();
  jrl::Results result(this->dataset_.name(), this->name_, robots);
  for (auto& rid : this->dataset_.robots()) {
    auto types = this->dataset_.initializationWithTypes(rid).types;
    result.robot_solutions[rid] = jrl::TypedValues(optimized_values[rid], types);
  }
  return BatchIterResults(result, 0); // NOT TRUE BUT we never use this number
}
/*********************************************************************************************************************/
template <class POSE_TYPE>
bool DDFSAM<POSE_TYPE>::isConverged() {
  return true;
}

}  // namespace batch_runners