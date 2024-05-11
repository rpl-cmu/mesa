
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <jrl/Dataset.h>
#include <jrl/Results.h>
#include <jrl/Types.h>

#include "batch_runners/MBADMMRunner.h"

namespace batch_runners {

/*********************************************************************************************************************/
template <class POSE_TYPE>
BatchIterResults MBADMMRunner<POSE_TYPE>::init() {
  // Construct the Multi-Block ADMM Solver
  mbadmm_solver_ = std::make_shared<ADMM<POSE_TYPE>>(
      0.1,                // rho: penalty parameter in augmented lagrangian
      10,                 // mu: parameter used to decide whether to increase or not the rho (adaptive penalty)
      2.0,                // tau:if we have to increase rho, we do rho <- rho * tau, otherwise rho <- rho / tau
      100000,             // maxIter [NOT USED]
      parallel_admm_,     // isParallel,
      adaptive_penalty_,  // adaptivePenalty
      false,              // useFlaggedInitialization,
      true,               // computeSubgraphGradient,
      0.1,                // min_p_res,
      0.1,                // min_d_res,
      "./"                // result dir [NOT USED]
  );
  // Set the ADMM Hyper Parameters
  mbadmm_solver_->setSolver(ADMM<POSE_TYPE>::Solver::GN);

  // Construct the graphs, estimates, format expected
  gtsam::NonlinearFactorGraph joint_graph;
  std::vector<gtsam::NonlinearFactorGraph> subgraphs;
  std::vector<gtsam::Values> sub_initials;
  std::map<char, size_t> robot_id_to_idx_map;
  size_t robot_count = 0;
  for (auto& rid : this->dataset_.robots()) {
    robot_id_to_idx_map[rid] = robot_count++;
    subgraphs.push_back(gtsam::NonlinearFactorGraph());
    sub_initials.push_back(this->dataset_.initialization(rid));

    // Accumulate all measurements
    for (auto& entry : this->dataset_.measurements(rid)) {
      subgraphs.back().push_back(entry.measurements);
      joint_graph.push_back(entry.measurements);
    }
  }

  // Accumulate the separators into the expected format
  std::set<gtsam::Key> seen_set;
  std::set<std::pair<char, char>> comm_edges;
  std::vector<size_t> separators;
  for (auto& rid : this->dataset_.robots()) {
    for (gtsam::Key key : this->dataset_.initialization(rid).keys()) {
      char key_rid = gtsam::Symbol(key).chr();
      if (key_rid != rid && seen_set.count(key) == 0) {
        // Mark the ij connection
        separators.push_back(key);
        separators.push_back(robot_id_to_idx_map[key_rid]);
        separators.push_back(robot_id_to_idx_map[rid]);

        seen_set.insert(key);
        comm_edges.insert(std::make_pair(std::min(rid, key_rid), std::max(rid, key_rid)));
      }
    }
  }
  // Record the number of communication edges
  // This is the number of communications per round the algorithm requires
  number_communication_edges_ = comm_edges.size();

  // Load the dataset into the solver
  mbadmm_solver_->load(subgraphs, sub_initials, separators, joint_graph);

  // return the initialization
  std::vector<char> robots = this->dataset_.robots();
  jrl::Results result(this->dataset_.name(), this->name_, robots);
  for (size_t i = 0; i < robots.size(); i++) {
    auto types = this->dataset_.initializationWithTypes(robots[i]).types;
    result.robot_solutions[robots[i]] = jrl::TypedValues(mbadmm_solver_->subinitials_[i], types);
  }
  return BatchIterResults(result, 0);
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
BatchIterResults MBADMMRunner<POSE_TYPE>::iterate() {
  std::vector<char> robots = this->dataset_.robots();

  // Cache the prev estimates
  std::vector<gtsam::Values> prev_estimates;
  for (size_t i = 0; i < robots.size(); i++) {
    prev_estimates.push_back(mbadmm_solver_->subinitials_[i]);
  }

  boost::tie(p_res_, d_res_) = mbadmm_solver_->iterate(iter_count_++);

  // update penalty parameter, when using adaptivePenalty
  if (mbadmm_solver_->adaptivePenalty_ && p_res_ > mbadmm_solver_->mu_ * d_res_) {
    mbadmm_solver_->rho_ = std::min(mbadmm_solver_->rho_ * mbadmm_solver_->tau_,
                                    1e6);  // increase penalty, to try to reduce primal infeasibility
  }
  if (mbadmm_solver_->adaptivePenalty_ && d_res_ > mbadmm_solver_->mu_ * p_res_) {
    mbadmm_solver_->rho_ = std::max(mbadmm_solver_->rho_ / mbadmm_solver_->tau_, 1e-3);  // reduce penalty
  }
  std::cout << mbadmm_solver_->rho_ << std::endl;
  // Aggregate the results

  jrl::Results result(this->dataset_.name(), this->name_, robots);
  for (size_t i = 0; i < robots.size(); i++) {
    auto types = this->dataset_.initializationWithTypes(robots[i]).types;
    result.robot_solutions[robots[i]] = jrl::TypedValues(mbadmm_solver_->subinitials_[i], types);
  }

  double norm = 0;
  for (size_t i = 0; i < robots.size(); i++) {
    norm += prev_estimates[i].localCoordinates(mbadmm_solver_->subinitials_[i]).norm();
  }

  converged_ = (p_res_ < 0.01 && d_res_ < 0.01);  // Strict convergence Criteria from paper

  return BatchIterResults(result, number_communication_edges_);
}
/*********************************************************************************************************************/
template <class POSE_TYPE>
bool MBADMMRunner<POSE_TYPE>::isConverged() {
  return converged_;
}

}  // namespace batch_runners