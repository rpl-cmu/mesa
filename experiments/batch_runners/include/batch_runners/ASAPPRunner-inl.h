#pragma once
#include "batch_runners/ASAPPRunner.h"
namespace batch_runners {
/*********************************************************************************************************************/
template <class POSE_TYPE>
std::pair<double, double> ASAPPRunner<POSE_TYPE>::convertGaussianCovarianceToKappaTau(
    const gtsam::SharedNoiseModel& noise_model) const {
  // Extract the Noise Model
  double translation_noise_param_tau;
  double rotation_noise_param_kappa;
  gtsam::SharedGaussian gaussian_model = boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(noise_model);
  gtsam::Matrix covariance = gaussian_model->covariance();
  // Because of diff tangent space params we need to check type to parse the noise models
  if (std::is_same<POSE_TYPE, gtsam::Pose2>::value) {
    double rot_cov = covariance(2, 2);
    gtsam::Matrix trans_cov = covariance.block<2, 2>(0, 0);
    // From DPGO read_g2o_file
    translation_noise_param_tau = 2.0 / trans_cov.trace();
    rotation_noise_param_kappa = rot_cov;
  } else {
    gtsam::Matrix rot_cov = covariance.block<3, 3>(0, 0);
    gtsam::Matrix trans_cov = covariance.block<3, 3>(3, 3);
    // From DPGO read_g2o_file
    translation_noise_param_tau = 3.0 / trans_cov.trace();
    rotation_noise_param_kappa = 3.0 / (2.0 * rot_cov.trace());
  }

  return std::make_pair(rotation_noise_param_kappa, translation_noise_param_tau);
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
DPGO::PriorSEMeasurement ASAPPRunner<POSE_TYPE>::convertPriorFactorToDPGOFormat(
    gtsam::NonlinearFactor::shared_ptr factor, const std::map<char, size_t>& robot_char_to_idx_map,
    DPGO::Matrix liftingMatrix) const {
  // Cast to proper factor
  typename gtsam::PriorFactor<POSE_TYPE>::shared_ptr prior_factor =
      boost::dynamic_pointer_cast<gtsam::PriorFactor<POSE_TYPE>>(factor);

  // Extract the pose indetifier information
  gtsam::Symbol sym(prior_factor->key());

  // Convert the noise model
  double translation_noise_param_tau;
  double rotation_noise_param_kappa;
  std::tie(rotation_noise_param_kappa, translation_noise_param_tau) =
      convertGaussianCovarianceToKappaTau(prior_factor->noiseModel());

  // Generate the Measurement
  POSE_TYPE prior = prior_factor->prior();
  return DPGO::PriorSEMeasurement(robot_char_to_idx_map.at(sym.chr()), sym.index(), prior.rotation().matrix(),
                                  prior.translation(), liftingMatrix, rotation_noise_param_kappa,
                                  translation_noise_param_tau);
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
DPGO::RelativeSEMeasurement ASAPPRunner<POSE_TYPE>::convertBetweenFactorToDPGOFormat(
    gtsam::NonlinearFactor::shared_ptr factor, const std::map<char, size_t>& robot_char_to_idx_map) const {
  // Cast to proper factor
  typename gtsam::BetweenFactor<POSE_TYPE>::shared_ptr between_factor =
      boost::dynamic_pointer_cast<gtsam::BetweenFactor<POSE_TYPE>>(factor);

  // Extract the pose indetifier information
  gtsam::KeyVector keys = between_factor->keys();
  gtsam::Symbol ka(keys.front());
  gtsam::Symbol kb(keys.back());

  // Convert the noise model
  double translation_noise_param_tau;
  double rotation_noise_param_kappa;
  std::tie(rotation_noise_param_kappa, translation_noise_param_tau) =
      convertGaussianCovarianceToKappaTau(between_factor->noiseModel());

  // Generate the Measurement
  POSE_TYPE measure = between_factor->measured();
  return DPGO::RelativeSEMeasurement(robot_char_to_idx_map.at(ka.chr()), robot_char_to_idx_map.at(kb.chr()), ka.index(),
                                     kb.index(), measure.rotation().matrix(), measure.translation(),
                                     rotation_noise_param_kappa, translation_noise_param_tau);
}
/*********************************************************************************************************************/
template <class POSE_TYPE>
typename ASAPPRunner<POSE_TYPE>::DPGODataset ASAPPRunner<POSE_TYPE>::convertDatasetToDPGOFormat(
    jrl::Dataset dataset, DPGO::Matrix liftingMatrix) const {
  // Construct a mapping between robot char names and their index
  std::vector<char> robot_ids = dataset.robots();
  std::map<char, size_t> robot_char_to_idx_map;
  for (size_t i = 0; i < robot_ids.size(); i++) {
    robot_char_to_idx_map[robot_ids[i]] = i;
  }

  // Initialize measurement vectors for each robot
  DPGODataset output_dataset;
  for (char rid : robot_ids) {
    output_dataset.priors.push_back(std::vector<DPGO::PriorSEMeasurement>());
    output_dataset.odometry.push_back(std::vector<DPGO::RelativeSEMeasurement>());
    output_dataset.private_loop_closures.push_back(std::vector<DPGO::RelativeSEMeasurement>());
    output_dataset.public_loop_closures.push_back(std::vector<DPGO::RelativeSEMeasurement>());
  }

  // For each robot add its measurements to the output dataset
  for (char rid : robot_ids) {
    for (auto& factor : dataset.factorGraph(rid)) {
      gtsam::KeyVector keys = factor->keys();
      if (keys.size() == 1) {  // PRIOR
        gtsam::Symbol key(keys.front());
        DPGO::PriorSEMeasurement prior = convertPriorFactorToDPGOFormat(factor, robot_char_to_idx_map, liftingMatrix);
        output_dataset.priors[robot_char_to_idx_map[key.chr()]].push_back(prior);
      } else if (keys.size() == 2) {  // BETWEEN MEASURE
        gtsam::Symbol ka(keys.front());
        gtsam::Symbol kb(keys.back());
        size_t ridx_a = robot_char_to_idx_map[ka.chr()];
        size_t ridx_b = robot_char_to_idx_map[kb.chr()];

        DPGO::RelativeSEMeasurement measure = convertBetweenFactorToDPGOFormat(factor, robot_char_to_idx_map);

        // Determine what kind of factor this is
        if (ka.chr() == kb.chr()) {            // Private Measuremet
          if (ka.index() + 1 == kb.index()) {  // Odometry
            output_dataset.odometry[ridx_a].push_back(measure);
          } else {  // Private Loop
            output_dataset.private_loop_closures[ridx_a].push_back(measure);
          }
        } else {  // Public Loop
          output_dataset.public_loop_closures[ridx_a].push_back(measure);
          output_dataset.public_loop_closures[ridx_b].push_back(measure);
        }
      }
    }
  }
  return output_dataset;
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
DPGO::Matrix ASAPPRunner<POSE_TYPE>::convertValuesToMatrix(const char rid, const gtsam::Values& values) const {
  gtsam::Values filtered_anchor_frame_poses;
  for (const auto& key : values.keys()) {
    if (gtsam::Symbol(key).chr() == rid) {
      POSE_TYPE pose = values.at<POSE_TYPE>(key);
      filtered_anchor_frame_poses.insert(key, pose);
    }
  }

  size_t num_poses = filtered_anchor_frame_poses.size();
  DPGO::Matrix output_matrix(pose_dim_, (pose_dim_ + 1) * num_poses);
  for (size_t i = 0; i < num_poses; i++) {
    POSE_TYPE pose = filtered_anchor_frame_poses.at<POSE_TYPE>(gtsam::symbol(rid, i));
    output_matrix.block(0, i * (pose_dim_ + 1), pose_dim_, pose_dim_) = pose.rotation().matrix();
    output_matrix.block(0, i * (pose_dim_ + 1) + pose_dim_, pose_dim_, 1) = pose.translation();
  }
  return output_matrix;
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
gtsam::Values ASAPPRunner<POSE_TYPE>::convertMatrixToValues(const char rid, const DPGO::Matrix& matrix) const {
  size_t num_poses = matrix.cols() / (pose_dim_ + 1);
  gtsam::Values output_values;
  for (size_t i = 0; i < num_poses; i++) {
    typename POSE_TYPE::Rotation rot(matrix.block(0, i * (pose_dim_ + 1), pose_dim_, pose_dim_));
    typename POSE_TYPE::Translation trans = matrix.block(0, i * (pose_dim_ + 1) + pose_dim_, pose_dim_, 1);
    output_values.insert(gtsam::symbol(rid, i), POSE_TYPE(rot, trans));
  }
  return output_values;
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
BatchIterResults ASAPPRunner<POSE_TYPE>::init() {
  // Setup the DGS runner for computing the initialization
  dgs_runner_ = std::make_shared<DGSRunner<POSE_TYPE>>(this->dataset_, this->output_directory_);
  dgs_runner_->max_rotation_iters = params_.max_number_dgs_rotation_iters;
  dgs_runner_->max_translation_iters = params_.max_number_dgs_pose_iters;
  BatchIterResults init_results = dgs_runner_->init();
  init_results.results.method_name = this->name_;
  dgs_results_ = std::make_shared<BatchIterResults>(init_results);

  // Initialize the DPGO Agents ref: DPGO/MultiRobotExample.cpp
  // First we setup the Agent options
  size_t tangent_space_dim = gtsam::traits<POSE_TYPE>::GetDimension(POSE_TYPE());
  pose_dim_ = tangent_space_dim == 6 ? 3 : 2;  /// Pose dimension if tangent space =6 this iss a 3d pose else is 2d
  DPGO::PGOAgentParameters agent_params(pose_dim_, params_.relaxation_rank);
  agent_params.asynchronous = false;  // This is really a param for threading which we dont want to use
  agent_params.localOptimizationParams.method = params_.optimization_method;
  agent_params.localOptimizationParams.RGD_stepsize = params_.rgd_step_size;
  agent_params.localInitializationMethod = DPGO::InitializationMethod::Chordal;

  // Construct each of the agents
  std::vector<char> robots = this->dataset_.robots();
  for (unsigned robot_idx = 0; robot_idx < robots.size(); robot_idx++) {
    std::shared_ptr<DPGO::PGOAgent> agent = std::make_shared<DPGO::PGOAgent>(robot_idx, agent_params);
    // All agents share a special, common matrix called the 'lifting matrix' which the first agent will generate
    if (robot_idx > 0) {
      gtsam::Matrix M;
      dpgo_agents_[0]->getLiftingMatrix(M);
      agent->setLiftingMatrix(M);
    }
    dpgo_agents_.push_back(agent);
  }

  // Convert the Dataset to DPGO format
  gtsam::Matrix M;
  dpgo_agents_[0]->getLiftingMatrix(M);
  dpgo_dataset_ = convertDatasetToDPGOFormat(this->dataset_, M);

  // Set the agent's measurements
  for (unsigned robot_idx = 0; robot_idx < robots.size(); robot_idx++) {
    dpgo_agents_[robot_idx]->setMeasurements(dpgo_dataset_.priors[robot_idx], dpgo_dataset_.odometry[robot_idx],
                                             dpgo_dataset_.private_loop_closures[robot_idx],
                                             dpgo_dataset_.public_loop_closures[robot_idx]);
    dpgo_agents_[robot_idx]->initialize();
  }

  // Setup the sampling distributions
  uniform_robot_distribution_ =
      std::uniform_int_distribution<size_t>(0, robots.size() - 1);  // unform is inclusive of bounds
  /// @brief Bernoulli distribution on the success of communication between a robot and its neighbors
  bernoulli_communication_distribution_ = std::bernoulli_distribution(params_.neighbor_communication_prob);

  return *dgs_results_;
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
void ASAPPRunner<POSE_TYPE>::runDPGOCommunication(unsigned selected_robot_idx, unsigned other_robot_idx) {
  // Selected Robot retrieves poses from its the neighbor
  DPGO::PoseDict shared_poses;
  if (!dpgo_agents_[other_robot_idx]->getSharedPoseDict(shared_poses)) return;
  dpgo_agents_[selected_robot_idx]->setNeighborStatus(dpgo_agents_[other_robot_idx]->getStatus());
  dpgo_agents_[selected_robot_idx]->updateNeighborPoses(dpgo_agents_[other_robot_idx]->getID(), shared_poses);
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
BatchIterResults ASAPPRunner<POSE_TYPE>::getCurrentDPGOResults(size_t number_iter_comms) {
  std::vector<char> robots = this->dataset_.robots();
  std::map<char, gtsam::Values> robot_values;

  // Extract all robot self poses
  for (size_t robot_idx = 0; robot_idx < robots.size(); robot_idx++) {
    DPGO::Matrix trajectory;
    // NOTE: This name is not-representative the trajectory is actually in the global frame
    dpgo_agents_[robot_idx]->getTrajectoryInGlobalFrame(trajectory);
    robot_values[robots[robot_idx]] = convertMatrixToValues(robots[robot_idx], trajectory);
  }

  // Augment values with each robots saved neighbor poses to match JRL invariant
  for (size_t robot_idx = 0; robot_idx < robots.size(); robot_idx++) {
    std::vector<unsigned> neighbors = dpgo_agents_[robot_idx]->getNeighbors();
    for (unsigned neighbor_idx : neighbors) {
      // Note we get stuff from the neighbor but only the ids for shared poses, which the robot would have locally
      // So this does not require communication
      DPGO::PoseDict neighbor_public_poses;
      dpgo_agents_[neighbor_idx]->getSharedPoseDictWithNeighbor(neighbor_public_poses, robot_idx);

      for (auto& kvp : neighbor_public_poses) {
        unsigned neighbor_pose_index = kvp.first.frame_id;
        DPGO::Matrix current_estimate_of_neighbor;
        dpgo_agents_[robot_idx]->getNeighborPoseInGlobalFrame(neighbor_idx, neighbor_pose_index,
                                                              current_estimate_of_neighbor);
        typename POSE_TYPE::Rotation rot(current_estimate_of_neighbor.block(0, 0, pose_dim_, pose_dim_));
        typename POSE_TYPE::Translation trans = current_estimate_of_neighbor.block(0, pose_dim_, pose_dim_, 1);
        robot_values[robots[robot_idx]].insert(gtsam::symbol(robots[neighbor_idx], neighbor_pose_index),
                                               POSE_TYPE(rot, trans));
      }
    }
  }

  // Compose jrl Results
  std::map<char, jrl::TypedValues> robot_solutions;
  for (char& rid : robots) {
    robot_solutions[rid] = jrl::TypedValues(robot_values[rid], this->dataset_.initializationWithTypes(rid).types);
  }
  return BatchIterResults(jrl::Results(this->dataset_.name(), this->name_, robots, robot_solutions), number_iter_comms);
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
BatchIterResults ASAPPRunner<POSE_TYPE>::iterateDPGO(size_t selected_robot_idx) {
  // Selected robot requests public poses from others
  size_t number_iteration_comms = 0;
  std::vector<unsigned> neighbors = dpgo_agents_[selected_robot_idx]->getNeighbors();
  for (unsigned neighbor_idx : neighbors) {
    // Sample to see if communication occurs
    if (bernoulli_communication_distribution_(rng_)) {
      number_iteration_comms++;
      runDPGOCommunication(selected_robot_idx, neighbor_idx);
    }
  }

  // With the updated poses the selected robot optimizes
  if (params_.optimization_method == DPGO::ROptParameters::ROptMethod::RGD) {
    // If RGD iterate the configured number of times
    for (size_t opt_iter = 0; opt_iter < params_.number_rgd_iters; opt_iter++) {
      dpgo_agents_[selected_robot_idx]->iterate(true);
    }
  } else {
    // If Trust region iterate only once
    dpgo_agents_[selected_robot_idx]->iterate(true);
  }

  // Compose the current DPGO results from each agent and return
  return getCurrentDPGOResults(number_iteration_comms);
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
void ASAPPRunner<POSE_TYPE>::updateConvergence(double step_norm) {
  std::vector<char> robots = this->dataset_.robots();
  // Add the new step norm
  norm_history_.push_back(step_norm);

  // If the history is now too long pop the oldest estimate
  if (norm_history_.size() > robots.size()) {
    norm_history_.pop_front();
  }

  // If the deque is long enough check for convergence
  if (norm_history_.size() == robots.size()) {
    double total_norm = 0.0;
    for (double sn : norm_history_) total_norm += sn;
    converged_ = total_norm / norm_history_.size() <= params_.norm_convergence_threshold;
  }
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
BatchIterResults ASAPPRunner<POSE_TYPE>::iterate() {
  // If our dgs is not done keep running dgs, and stop early
  if (!dgs_runner_->isConverged()) {
    BatchIterResults iter_results = dgs_runner_->iterate();
    dgs_results_ = std::make_shared<BatchIterResults>(iter_results);
    return *dgs_results_;
  }

  // DGS is done but we have not initialized the agents with the DGS result
  std::vector<char> robots = this->dataset_.robots();
  if (!dpgo_initialized_) {
    for (size_t i = 0; i < robots.size(); i++) {
      // Convert Values to pose array form
      DPGO::Matrix robot_init_matrix =
          convertValuesToMatrix(robots[i], dgs_results_->results.robot_solutions[robots[i]].values);
      // Lift the matrix and configure the agent with the lifted value
      DPGO::Matrix lifting_matrix;
      dpgo_agents_[i]->getLiftingMatrix(lifting_matrix);
      DPGO::Matrix lifted_robot_init_matrix = lifting_matrix * robot_init_matrix;
      dpgo_agents_[i]->setX(lifted_robot_init_matrix);

      // Let the anchor to identity as we treat all poses as they are in the global frame
      DPGO::Matrix robot_anchor_matrix(pose_dim_, pose_dim_ + 1);
      robot_anchor_matrix.block(0, 0, pose_dim_, pose_dim_) = DPGO::Matrix::Identity(pose_dim_, pose_dim_);
      robot_anchor_matrix.block(0, pose_dim_, pose_dim_, 1) = DPGO::Matrix::Zero(pose_dim_, 1);
      dpgo_agents_[i]->setGlobalAnchor(lifting_matrix * robot_anchor_matrix);
    }

    // Initial round of communication so robots have access to required data
    for (size_t i = 0; i < robots.size(); i++) {
      for (size_t j = 0; j < robots.size(); j++) {
        if (i != j) runDPGOCommunication(i, j);
      }
    }

    dpgo_initialized_ = true;
  }

  // Iterate DPO
  size_t selected_robot_idx = uniform_robot_distribution_(rng_);
  BatchIterResults dpgo_iter_results = iterateDPGO(selected_robot_idx);

  // Check Convergence Criteria
  gtsam::Values selected_robot_solution = dpgo_iter_results.results.robot_solutions[robots[selected_robot_idx]].values;
  if (prev_iter_results) {
    gtsam::Values prev_selected_robot_solution =
        prev_iter_results->results.robot_solutions[robots[selected_robot_idx]].values;
    double norm = selected_robot_solution.localCoordinates(prev_selected_robot_solution).norm();
    updateConvergence(norm);
  }
  prev_iter_results = std::make_shared<BatchIterResults>(dpgo_iter_results);

  return dpgo_iter_results;
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
bool ASAPPRunner<POSE_TYPE>::isConverged() {
  return converged_;
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
size_t ASAPPRunner<POSE_TYPE>::iterationsPerFullComms() {
  return this->dataset_.robots().size();
}

}  // namespace batch_runners