#pragma once

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include <boost/math/distributions.hpp>
#include <iterator>

#include "mesa/BatchMESA.h"
#include "mesa/CorrectPrior.h"

/*********************************************************************************************************************/
template <class POSE_TYPE>
POSE_TYPE baseInterpolateSLERP(const POSE_TYPE& pa, const POSE_TYPE& pb, double alpha) {
  return gtsam::traits<POSE_TYPE>::Retract(pa, alpha * gtsam::traits<POSE_TYPE>::Local(pa, pb));
}

template <class POSE_TYPE>
POSE_TYPE baseInterpolateSPLIT(const POSE_TYPE& start_pose, const POSE_TYPE& end_pose, double alpha) {
  typename POSE_TYPE::Rotation rs = start_pose.rotation();
  typename POSE_TYPE::Rotation re = end_pose.rotation();
  typename POSE_TYPE::Rotation interp_rot = rs.compose(gtsam::traits<typename POSE_TYPE::Rotation>::Expmap(
      alpha * gtsam::traits<typename POSE_TYPE::Rotation>::Logmap(rs.inverse().compose(re))));
  typename POSE_TYPE::Translation interp_trans =
      start_pose.translation() + (alpha * (end_pose.translation() - start_pose.translation()));
  return POSE_TYPE(interp_rot, interp_trans);
}

// Need specializations for linear cases because they dont actually have rotations to separate out
template <>
gtsam::Point2 baseInterpolateSPLIT(const gtsam::Point2& start_pose, const gtsam::Point2& end_pose, double alpha) {
  return baseInterpolateSLERP(start_pose, end_pose, alpha);
}
template <>
gtsam::Point3 baseInterpolateSPLIT(const gtsam::Point3& start_pose, const gtsam::Point3& end_pose, double alpha) {
  return baseInterpolateSLERP(start_pose, end_pose, alpha);
}

/*********************************************************************************************************************/

template <class POSE_TYPE, class Z_TYPE, class BIASED_PRIOR_TYPE, class INV_BIASED_PRIOR_TYPE>
BatchMESA<POSE_TYPE, Z_TYPE, BIASED_PRIOR_TYPE, INV_BIASED_PRIOR_TYPE>::BatchMESA(std::vector<char> robots,
                                                                                        RobotGraphs robot_graphs,
                                                                                        RobotValues robot_estimates,
                                                                                        MESAParams params)
    : robots_(robots), robot_base_graphs_(robot_graphs), robot_estimates_(robot_estimates), params_(params) {}

/*********************************************************************************************************************/
template <class POSE_TYPE, class Z_TYPE, class BIASED_PRIOR_TYPE, class INV_BIASED_PRIOR_TYPE>
RobotSharedVariables BatchMESA<POSE_TYPE, Z_TYPE, BIASED_PRIOR_TYPE, INV_BIASED_PRIOR_TYPE>::parseSharedVariables(
    std::vector<char> robots, RobotGraphs robot_graphs) {
  RobotSharedVariables result;
  // Setup the structure
  for (auto& rid : robots) {
    result[rid] = std::map<char, std::set<gtsam::Key>>();
    for (auto& oid : robots) {
      result[rid][oid] = std::set<gtsam::Key>();
    }
  }

  // Fill in Structure
  for (auto& rid : robots) {
    for (auto factor : robot_graphs[rid]) {
      for (auto& key : factor->keys()) {
        gtsam::Key key_rid = gtsam::Symbol(key).chr();
        if (key_rid != rid) {
          result[rid][key_rid].insert(key);
          result[key_rid][rid].insert(key);
        }
      }
    }
  }
  return result;
}

/*********************************************************************************************************************/
template <class POSE_TYPE, class Z_TYPE, class BIASED_PRIOR_TYPE, class INV_BIASED_PRIOR_TYPE>
std::set<CommunicationEdge>
BatchMESA<POSE_TYPE, Z_TYPE, BIASED_PRIOR_TYPE, INV_BIASED_PRIOR_TYPE>::parseCommunicationNetwork(
    std::vector<char> robots, RobotSharedVariables robot_shared_variables) {
  std::set<CommunicationEdge> result;
  for (auto rid : robots) {
    for (auto oid : robots) {
      // If these two robots share any variables
      if (robot_shared_variables[rid][oid].size() > 0) {
        result.insert(std::make_pair(std::min(rid, oid), std::max(rid, oid)));
        // Note: allow double insert, std::set will only maintain unique pairs
      }
    }
  }
  return result;
}

/*********************************************************************************************************************/
template <class POSE_TYPE, class Z_TYPE, class BIASED_PRIOR_TYPE, class INV_BIASED_PRIOR_TYPE>
RobotGraphs BatchMESA<POSE_TYPE, Z_TYPE, BIASED_PRIOR_TYPE, INV_BIASED_PRIOR_TYPE>::constructDeterminedGraphs(
    std::vector<char> robots, RobotGraphs robot_base_graphs, RobotValues robot_estimates,
    RobotSharedVariables robot_shared_variables, bool prior_shared_variables, gtsam::Vector shared_prior_sigmas) {
  // Initialize the result
  RobotGraphs determined_graphs;

  // For each robot solve independently
  for (auto& rid : robots) {
    gtsam::NonlinearFactorGraph determined_factor_graph = robot_base_graphs[rid].clone();

    // Add weak priors to any shared variable if configured
    if (prior_shared_variables) {
      std::set<gtsam::Key> seen_shared;
      for (auto& oid : robots) {
        for (auto& key : robot_shared_variables[rid][oid]) {
          if (seen_shared.count(key) == 0) {
            determined_factor_graph.emplace_shared<CorrectPrior<POSE_TYPE>>(
                key, robot_estimates[rid].at<POSE_TYPE>(key),
                gtsam::noiseModel::Isotropic::Sigmas(shared_prior_sigmas));
            seen_shared.insert(key);
          }
        }
      }
    }

    determined_graphs[rid] = determined_factor_graph;
  }

  return determined_graphs;
}

/*********************************************************************************************************************/
template <class POSE_TYPE, class Z_TYPE, class BIASED_PRIOR_TYPE, class INV_BIASED_PRIOR_TYPE>
void BatchMESA<POSE_TYPE, Z_TYPE, BIASED_PRIOR_TYPE, INV_BIASED_PRIOR_TYPE>::updateSharedVariables(
    CommunicationEdge comm_edge) {
  for (auto shared_key : robot_shared_variables_[comm_edge.first][comm_edge.second]) {
    // Use the child class to optimize the estimate
    std::pair<Z_TYPE, gtsam::Matrix> value_info_pair = computeNewZ(
        robot_estimates_[comm_edge.first].at<POSE_TYPE>(shared_key),
        edge_robot_dual_variables_[comm_edge][comm_edge.first].at<gtsam::Vector>(shared_key),
        robot_base_marginals_[comm_edge.first].marginalInformation(shared_key),
        robot_estimates_[comm_edge.second].at<POSE_TYPE>(shared_key),
        edge_robot_dual_variables_[comm_edge][comm_edge.second].at<gtsam::Vector>(shared_key),
        robot_base_marginals_[comm_edge.second].marginalInformation(shared_key), edge_beta_variables_[comm_edge]);

    // Update the class fields with the new information
    if (edge_shared_estimates_[comm_edge].exists(shared_key)) {
      edge_shared_estimates_[comm_edge].update(shared_key, value_info_pair.first);
    } else {
      edge_shared_estimates_[comm_edge].insert(shared_key, value_info_pair.first);
    }
    edge_shared_noise_models_[comm_edge][shared_key] = gtsam::noiseModel::Gaussian::Information(value_info_pair.second);
  }
}

/*********************************************************************************************************************/
template <class POSE_TYPE, class Z_TYPE, class BIASED_PRIOR_TYPE, class INV_BIASED_PRIOR_TYPE>
void BatchMESA<POSE_TYPE, Z_TYPE, BIASED_PRIOR_TYPE, INV_BIASED_PRIOR_TYPE>::updateRobotEstimateAndMarginals(
    char rid) {
  gtsam::NonlinearFactorGraph biased_factor_graph;
  biased_factor_graph.push_back(robot_base_graphs_[rid]);

  for (auto oid : robots_) {
    CommunicationEdge comm_robots = std::make_pair(std::min(rid, oid), std::max(rid, oid));
    for (auto sk : robot_shared_variables_[rid][oid]) {
      double beta = edge_beta_variables_[comm_robots];
      biased_factor_graph.push_back(BIASED_PRIOR_TYPE(
          sk, edge_shared_estimates_[comm_robots].at<Z_TYPE>(sk), edge_shared_noise_models_[comm_robots][sk],
          edge_robot_dual_variables_[comm_robots][rid].at<gtsam::Vector>(sk), beta));
    }
  }

  // Do the optimization and update relevant info
  gtsam::LevenbergMarquardtOptimizer optimizer(biased_factor_graph, robot_estimates_[rid]);
  robot_estimates_[rid] = optimizer.optimize();
  robot_base_marginals_[rid] = gtsam::Marginals(robot_determined_graphs_[rid], robot_estimates_[rid]);
}

/*********************************************************************************************************************/
template <class POSE_TYPE, class Z_TYPE, class BIASED_PRIOR_TYPE, class INV_BIASED_PRIOR_TYPE>
void BatchMESA<POSE_TYPE, Z_TYPE, BIASED_PRIOR_TYPE, INV_BIASED_PRIOR_TYPE>::updateDualVariables(
    CommunicationEdge comm_edge) {
  double beta = edge_beta_variables_[comm_edge];
  for (gtsam::Key shared_key : robot_shared_variables_[comm_edge.first][comm_edge.second]) {
    // Get the Poses for the two robots
    POSE_TYPE pa = robot_estimates_[comm_edge.first].at<POSE_TYPE>(shared_key);
    POSE_TYPE pb = robot_estimates_[comm_edge.second].at<POSE_TYPE>(shared_key);
    // Get the Dual Variables for the two robots
    gtsam::Vector a_dual = edge_robot_dual_variables_[comm_edge][comm_edge.first].at<gtsam::Vector>(shared_key);
    gtsam::Vector b_dual = edge_robot_dual_variables_[comm_edge][comm_edge.second].at<gtsam::Vector>(shared_key);
    // Initialize variables for new dual Values
    gtsam::Vector new_a_dual;
    gtsam::Vector new_b_dual;

    // Handle Chourdary Special Case first
    if (params_.dual_compute_target == MESAParams::DualComputeTarget::CHOUDHARY_TARGET) {
      // Compute the dual for robot a using robot b as target
      new_a_dual = computeNewDualPose(a_dual, beta, pa, pb);
      new_b_dual = -new_a_dual;  // Robot b gets incorrect but useful negation

    }
    // Standard approach, with configurable target
    else {
      // Get the target for each of the robots using the configured method
      if (params_.dual_compute_target == MESAParams::DualComputeTarget::SHARED_ESTIMATE) {
        Z_TYPE dual_target_robot_a = edge_shared_estimates_[comm_edge].at<Z_TYPE>(shared_key);
        Z_TYPE dual_target_robot_b = edge_shared_estimates_[comm_edge].at<Z_TYPE>(shared_key);
        // Compute the New Dual Estimates using the configured target
        new_a_dual = computeNewDualShared(a_dual, beta, pa, dual_target_robot_a);
        new_b_dual = computeNewDualShared(b_dual, beta, pb, dual_target_robot_b);
      } else if (params_.dual_compute_target == MESAParams::DualComputeTarget::OTHER_ESTIMATE) {
        new_a_dual = computeNewDualPose(a_dual, beta, pa, pb);
        new_b_dual = computeNewDualPose(b_dual, beta, pb, pa);
      } else if (params_.dual_compute_target == MESAParams::DualComputeTarget::UNWEIGHTED_SLERP) {
        Z_TYPE slerp_z = interpolateSLERP(pa, pb, 0.5);
        new_a_dual = computeNewDualShared(a_dual, beta, pa, slerp_z);
        new_b_dual = computeNewDualShared(b_dual, beta, pb, slerp_z);
      }
    }

    // Update the dual variables in memory
    edge_robot_dual_variables_[comm_edge][comm_edge.first].update(shared_key, new_a_dual);
    edge_robot_dual_variables_[comm_edge][comm_edge.second].update(shared_key, new_b_dual);
  }
}

/*********************************************************************************************************************/
template <class POSE_TYPE, class Z_TYPE, class BIASED_PRIOR_TYPE, class INV_BIASED_PRIOR_TYPE>
std::pair<Z_TYPE, gtsam::Matrix> BatchMESA<POSE_TYPE, Z_TYPE, BIASED_PRIOR_TYPE, INV_BIASED_PRIOR_TYPE>::computeNewZ(
    const POSE_TYPE& pa, const gtsam::Vector& dual_a, const gtsam::Matrix& info_a, const POSE_TYPE& pb,
    const gtsam::Vector& dual_b, const gtsam::Matrix& info_b, const double beta) const {
  // Setup the problem
  double interp_weight = 0.5;
  if (params_.weight_z_compute) {
    interp_weight = 1.0 - (info_a.norm() / (info_a.norm() + info_b.norm()));
  }

  // Construct the noise models depending on configuration
  gtsam::SharedNoiseModel noise_model_a;
  gtsam::SharedNoiseModel noise_model_b;
  if (params_.weight_z_compute) {
    noise_model_a = gtsam::noiseModel::Gaussian::Information(info_a);
    noise_model_b = gtsam::noiseModel::Gaussian::Information(info_b);
  } else {
    size_t d = gtsam::traits<POSE_TYPE>::GetDimension(pa);
    noise_model_a = gtsam::noiseModel::Unit::Create(d);
    noise_model_b = gtsam::noiseModel::Unit::Create(d);
  }

  // Construct the Graph
  gtsam::NonlinearFactorGraph graph;
  // Note Because we are minimizing wrt z use the INVERSE PRIORS
  graph.emplace_shared<INV_BIASED_PRIOR_TYPE>(0, pa, noise_model_a, dual_a, beta);
  graph.emplace_shared<INV_BIASED_PRIOR_TYPE>(0, pb, noise_model_b, dual_b, beta);

  // Solve using the configured computation method
  Z_TYPE solution_z;
  if (params_.z_compute_method == MESAParams::ComputeZMethod::INTERPOLATE_SPLIT) {
    solution_z = interpolateSPLIT(pa, pb, interp_weight);
  } else if (params_.z_compute_method == MESAParams::ComputeZMethod::INTERPOLATE_SLERP) {
    solution_z = interpolateSLERP(pa, pb, interp_weight);
  } else if (params_.z_compute_method == MESAParams::ComputeZMethod::OPTIMIZE) {
    // Use a SPLIT interpolation weighted appropriately to initialize the optimization
    gtsam::Values initialization;
    Z_TYPE init_z = interpolateSPLIT(pa, pb, interp_weight);
    initialization.insert(0, init_z);
    // Solve the Optimization Problem
    gtsam::GaussNewtonOptimizer optimizer(graph, initialization);
    gtsam::Values optimized = optimizer.optimize();
    solution_z = optimized.at<Z_TYPE>(0);
  }

  // Compute the Marginal
  gtsam::Values solution_values;
  solution_values.insert(0, solution_z);
  gtsam::Marginals optimized_marginal(graph, solution_values);

  // Return the optimized value and the corresponding marginal information
  if (params_.weight_z_compute) {
    return std::make_pair(solution_z, optimized_marginal.marginalInformation(0));
  } else {
    size_t d = gtsam::traits<POSE_TYPE>::GetDimension(pa);
    return std::make_pair(solution_z, gtsam::Matrix::Identity(d, d));
  }
}

/*********************************************************************************************************************/
template <class POSE_TYPE, class Z_TYPE, class BIASED_PRIOR_TYPE, class INV_BIASED_PRIOR_TYPE>
RobotValues BatchMESA<POSE_TYPE, Z_TYPE, BIASED_PRIOR_TYPE, INV_BIASED_PRIOR_TYPE>::init() {
  // Initialize the Shared Variables
  robot_shared_variables_ = parseSharedVariables(robots_, robot_base_graphs_);

  // Initialize the Communication Network
  communication_network_ = parseCommunicationNetwork(robots_, robot_shared_variables_);
  std::cout << "Communication Network contains " << communication_network_.size() << " Links" << std::endl;
  for (auto pair : communication_network_) {
    std::cout << "Edge: " << pair.first << ", " << pair.second << std::endl;
  }

  // Generate the Distribution over the communication edges
  std::random_device dev;
  random_number_generator_ = std::mt19937(dev());
  communication_edge_distribution_ = std::uniform_int_distribution<int>(0, communication_network_.size() - 1);

  // Initialize the Determined Graphs
  robot_determined_graphs_ =
      constructDeterminedGraphs(robots_, robot_base_graphs_, robot_estimates_, robot_shared_variables_,
                                params_.prior_shared_vars_on_indep_solve, params_.shared_var_prior_sigmas);

  // Initialize the counts since last comm
  for (auto& rid : robots_) {
    // Initialize comm count
    robot_counts_since_last_comm_[rid] = std::map<CommunicationEdge, size_t>();
    for (auto& edge : communication_network_) {
      robot_counts_since_last_comm_[rid][edge] = 0;
    }
  }

  // Populate the edge counts and beta variables, shared_estimates, and dual variables
  for (const auto& comm_edge : communication_network_) {
    // Initialize beta
    edge_beta_variables_[comm_edge] = params_.beta_init;

    // Initialize the shared variable estimate structures
    edge_shared_noise_models_[comm_edge] = std::map<gtsam::Key, gtsam::SharedNoiseModel>();
    edge_shared_estimates_[comm_edge] = gtsam::Values();

    // Initialize the Dual Variable Structures
    edge_robot_dual_variables_[comm_edge] = std::map<char, gtsam::Values>();
    edge_robot_lie_dual_variables_[comm_edge] = std::map<char, gtsam::Values>();
    // Initialize all dual variables
    for (auto sk : robot_shared_variables_[comm_edge.first][comm_edge.second]) {
      size_t dim = getDualDim();
      // Note dont try to directly insert from the zero constructor, causes issues with gtsam instantiating a generic
      // value type for CWiseNullaryOp
      gtsam::Vector dual_first = gtsam::Vector::Zero(dim);
      gtsam::Vector dual_second = gtsam::Vector::Zero(dim);
      // Initialize the variant's dual variables
      edge_robot_dual_variables_[comm_edge][comm_edge.first].insert(sk, dual_first);
      edge_robot_dual_variables_[comm_edge][comm_edge.second].insert(sk, dual_second);

      // Initialize the Lie formulation dual variables for computing the dual residual
      edge_robot_lie_dual_variables_[comm_edge][comm_edge.first].insert(sk, dual_first);
      edge_robot_lie_dual_variables_[comm_edge][comm_edge.second].insert(sk, dual_second);
    }
  }

  // Note: Marginals, and shared variables are initialized during the first call to iterate!
  return robot_estimates_;
}

/*********************************************************************************************************************/
template <class POSE_TYPE, class Z_TYPE, class BIASED_PRIOR_TYPE, class INV_BIASED_PRIOR_TYPE>
RobotValues BatchMESA<POSE_TYPE, Z_TYPE, BIASED_PRIOR_TYPE, INV_BIASED_PRIOR_TYPE>::iterate() {
  // Cache Previous estimates
  RobotValues prev_estimates;
  for (auto& rid : robots_) {
    prev_estimates[rid] = robot_estimates_[rid];
  }

  if (first_iterate_) {
    first_iterate_ = false;
    // Optimize each robot individually
    for (auto& rid : robots_) {
      std::cout << "Indep Opt for robot: " << rid << std::endl;
      gtsam::LevenbergMarquardtOptimizer optimizer(robot_determined_graphs_[rid], robot_estimates_[rid]);
      robot_estimates_[rid] = optimizer.optimize();
    }

    // Initialize the marginals
    for (auto& rid : robots_) {
      std::cout << "Marginals for robot: " << rid << std::endl;
      robot_base_marginals_[rid] = gtsam::Marginals(robot_determined_graphs_[rid], robot_estimates_[rid]);
    }

    // run a round of communication to initialize all of the shared estimates
    for (auto comm_edge : communication_network_) {
      for (auto shared_key : robot_shared_variables_[comm_edge.first][comm_edge.second]) {
        char key_owner = gtsam::Symbol(shared_key).chr();
        if (key_owner == comm_edge.first) {
          POSE_TYPE value = robot_estimates_[comm_edge.first].at<POSE_TYPE>(shared_key);
          robot_estimates_[comm_edge.second].update(shared_key, value);
        } else {
          POSE_TYPE value = robot_estimates_[comm_edge.second].at<POSE_TYPE>(shared_key);
          robot_estimates_[comm_edge.first].update(shared_key, value);
        }
      }
      // Update the Shared Variable Estimates
      updateSharedVariables(comm_edge);
    }

  } else {
    // Get a random edge
    auto edge_iter = communication_network_.begin();
    int n = communication_edge_distribution_(random_number_generator_);
    std::advance(edge_iter, n);

    // Run an update on these variables
    preformCommunicationStep(*edge_iter);
  }

  // Check Convergence
  double norm = 0.0;
  for (auto rid : robots_) {
    norm += prev_estimates[rid].localCoordinates(robot_estimates_[rid]).norm();
  }
  std::cout << "\tDelta: " << norm << std::endl;
  updateConvergence(norm);

  // return the estimates from this step
  return robot_estimates_;
}

template <class POSE_TYPE, class Z_TYPE, class BIASED_PRIOR_TYPE, class INV_BIASED_PRIOR_TYPE>
void BatchMESA<POSE_TYPE, Z_TYPE, BIASED_PRIOR_TYPE, INV_BIASED_PRIOR_TYPE>::updateConvergence(double step_norm) {
  // Add the new step norm
  norm_history_.push_back(step_norm);

  // If the history is now too long pop the oldest estimate
  if (norm_history_.size() > robots_.size()) {
    norm_history_.pop_front();
  }

  // If the deque is long enough check for convergence
  if (norm_history_.size() == robots_.size()) {
    double total_norm = 0.0;
    for (double sn : norm_history_) total_norm += sn;
    converged_ = total_norm / norm_history_.size() <= params_.convergence_threshold;
  }
}

/*********************************************************************************************************************/
template <class POSE_TYPE, class Z_TYPE, class BIASED_PRIOR_TYPE, class INV_BIASED_PRIOR_TYPE>
bool BatchMESA<POSE_TYPE, Z_TYPE, BIASED_PRIOR_TYPE, INV_BIASED_PRIOR_TYPE>::updateCountsSinceLastComm(
    CommunicationEdge current_edge) {
  bool should_update_beta = robot_counts_since_last_comm_[current_edge.first][current_edge] >= robots_.size() - 1 &&
                            robot_counts_since_last_comm_[current_edge.second][current_edge] >= robots_.size() - 1;

  for (auto& edge : communication_network_) {
    if (edge == current_edge && should_update_beta) {
      robot_counts_since_last_comm_[current_edge.first][edge] = 0;
      robot_counts_since_last_comm_[current_edge.second][edge] = 0;
    } else {
      robot_counts_since_last_comm_[current_edge.first][edge]++;
      robot_counts_since_last_comm_[current_edge.second][edge]++;
    }
  }
  return should_update_beta;
}

/*********************************************************************************************************************/
template <class POSE_TYPE, class Z_TYPE, class BIASED_PRIOR_TYPE, class INV_BIASED_PRIOR_TYPE>
void BatchMESA<POSE_TYPE, Z_TYPE, BIASED_PRIOR_TYPE, INV_BIASED_PRIOR_TYPE>::preformCommunicationStep(
    CommunicationEdge comm_robots) {
  std::cout << "Run Step: " << comm_robots.first << " - " << comm_robots.second << "("
            << edge_beta_variables_[comm_robots] << ")" << std::endl;
  // Update the Communication count for this network edge
  bool should_update_beta = updateCountsSinceLastComm(comm_robots);

  // Run the Optimization [Local information only]
  updateRobotEstimateAndMarginals(comm_robots.first);
  updateRobotEstimateAndMarginals(comm_robots.second);

  // HERE is where the robots communicate each shared variable to each other
  // Update the Shared Variable Estimates [Local information only]
  updateSharedVariables(comm_robots);

  // Update the Dual Variables [Local information only]
  updateDualVariables(comm_robots);

  // Update the beta variables
  if (!params_.psuedo_sync_beta || should_update_beta) {
    edge_beta_variables_[comm_robots] *= params_.beta_multiplier_increase;
  }
}