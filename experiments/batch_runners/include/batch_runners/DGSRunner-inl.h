
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <jrl/Dataset.h>
#include <jrl/Results.h>
#include <jrl/Types.h>

#include <boost/optional/optional_io.hpp>

#include "batch_runners/DGSRunner.h"

namespace batch_runners {

/*********************************************************************************************************************/
template <class POSE_TYPE>
BatchIterResults DGSRunner<POSE_TYPE>::init() {
  std::map<char, gtsam::NonlinearFactorGraph> robot_graphs;
  std::vector<char> robots = this->dataset_.robots();
  for (size_t i = 0; i < robots.size(); i++) {
    char rid = robots[i];
    robot_graphs[rid] = gtsam::NonlinearFactorGraph();
    current_estimates_.push_back(gtsam::Values());

    // The example DGS code doesnt supply agents with estimates from other robots
    for (auto& key : this->dataset_.initialization(rid).keys()) {
      if (gtsam::Symbol(key).chr() == rid) {
        current_estimates_.back().insert(key, this->dataset_.initialization(rid).at(key));
      }
    }

    // Accumulate all measurements
    for (auto& entry : this->dataset_.measurements(rid)) {
      robot_graphs[rid].push_back(entry.measurements);
    }
  }

  /** DGS partitions over variables, rather than over measurements.
   * So for any inter robot measurement, we need to add it to both graphs of the robots it affects.
   *
   * This isnt clear from the code, but is in the data included in the distributed mapper code
   */
  std::map<char, gtsam::NonlinearFactorGraph> augmented_robot_graphs;
  for (auto& rid : robots) {
    augmented_robot_graphs[rid] = robot_graphs[rid].clone();
  }

  for (auto& rid : robots) {
    for (auto& factor : robot_graphs[rid]) {
      std::set<char> add_to_robot;
      for (auto& key : factor->keys()) {
        gtsam::Symbol sym(key);
        if (sym.chr() != rid) {
          add_to_robot.insert(sym.chr());
        }
      }

      for (auto& oid : add_to_robot) {
        augmented_robot_graphs[oid].push_back(factor);
      }
    }
  }

  for (size_t i = 0; i < robots.size(); i++) {
    char rid = robots[i];
    // Construct a distributed jacobi object with the given robot name
    distributed_mappers_.push_back(boost::make_shared<distributed_mapper::DistributedMapper>(rid));
    auto graph_vals = std::make_pair(boost::make_shared<gtsam::NonlinearFactorGraph>(augmented_robot_graphs[rid]),
                                     boost::make_shared<gtsam::Values>(current_estimates_[i]));
    distributed_mappers_.back()->setUseBetweenNoiseFlag(true);
    distributed_mappers_.back()->setVerbosity(distributed_mapper::DistributedMapper::Verbosity::DEBUG);
    distributed_mappers_.back()->loadSubgraphAndCreateSubgraphEdge(graph_vals);
    distributed_mappers_.back()->setFlaggedInit(false);
    distributed_mappers_.back()->setUpdateType(distributed_mapper::DistributedMapper::incUpdate);
    distributed_mappers_.back()->setGamma(1.0);
    distributed_mappers_.back()->updateInitialized(false);
    distributed_mappers_.back()->clearNeighboringRobotInit();
  }

  // Compute the Ordering
  for (char rid : robots) {
    robot_names_ += std::string(1, rid);
  }

  // Compute the Number of Communication edges
  std::set<std::pair<char, char>> comm_edges;
  for (auto& mapper : distributed_mappers_) {
    char r = mapper->robotName();
    for (char o : mapper->getNeighboringChars()) {
      comm_edges.insert(std::make_pair(std::min(r, o), std::max(r, o)));
    }
  }
  number_communication_edges_ = comm_edges.size();

  ordering_ = orderRobots(distributed_mappers_, robots.size(), robot_names_, false, false);

  jrl::Results result(this->dataset_.name(), this->name_, robots);
  for (auto& rid : robots) {
    result.robot_solutions[rid] = this->dataset_.initializationWithTypes(rid);
  }
  return BatchIterResults(result, 0);
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
bool DGSRunner<POSE_TYPE>::iterateRotations() {
  for (size_t robot : ordering_) {
    // Ask other robots for updated estimates and update it
    for (const gtsam::Values::ConstKeyValuePair& key_value : distributed_mappers_[robot]->neighbors()) {
      gtsam::Key key = key_value.key;

      // distributed_mappers_ only contains the robots that are currently communicating, so we check
      // if the neighbor keys is from one of these robots, if so, we update it (communication)
      char symbol = gtsam::symbolChr(key);
      size_t neighboringRobotId = robot_names_.find(symbol);
      if (neighboringRobotId != std::string::npos) {  // if the robot is actually communicating
        gtsam::Vector rotationEstimate = distributed_mappers_[neighboringRobotId]->linearizedRotationAt(key);
        distributed_mappers_[robot]->updateNeighborLinearizedRotations(
            key, rotationEstimate);  // this requires communication

        bool neighboringRobotInitialized = distributed_mappers_[neighboringRobotId]->isRobotInitialized();
        distributed_mappers_[robot]->updateNeighboringRobotInitialized(
            symbol, neighboringRobotInitialized);  // this requires communication
      } else {
        // Robot we are not communicating with are considered as optimized
        distributed_mappers_[robot]->updateNeighboringRobotInitialized(symbol, true);
      }
    }

    // Iterate
    distributed_mappers_[robot]->estimateRotation();  // optimization

    /*  Distributed Jacobi: updateType_ = postUpdate, gamma = 1
     *  Gauss Seidel: updateType_ = incUpdate, gamma = 1
     *  Jacobi Overrelax: updateType_ = postUpdate, gamma != 1
     *  Succ Overrelax: updateType_ = incUpdate, gamma != 1
     */
    if (distributed_mappers_[robot]->updateType_ == distributed_mapper::DistributedMapper::incUpdate) {
      distributed_mappers_[robot]->updateRotation();
    }

    // This robot is initialized
    distributed_mappers_[robot]->updateInitialized(true);

    // Setup the current estimates
    gtsam::Values current_rotation_estimate =
        gtsam::InitializePose3::normalizeRelaxedRotations(distributed_mappers_[robot]->linearizedRotations());
    for (auto key : current_estimates_[robot].keys()) {
      gtsam::Pose3 new_pose(current_rotation_estimate.at<gtsam::Rot3>(key),
                            current_estimates_[robot].at<gtsam::Pose3>(key).translation());
      current_estimates_[robot].update(key, new_pose);
    }
  }

  //  STOP IF
  // 1- change in the individual cost of each robot is small enough
  // 2- change in the estimate of each robot is small enough
  bool converged = true;
  for (size_t robot = 0; robot < this->dataset_.robots().size(); robot++) {
    std::cout << "\t" << robot << ": " << distributed_mappers_[robot]->latestChange() << std::endl;
    converged = converged && distributed_mappers_[robot]->latestChange() <= 1e-2;  // "Tight" Bound from paper
    std::cout << "\t"
              << "converged: " << distributed_mappers_[robot]->latestChange() << std::endl;
  }
  return converged;
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
bool DGSRunner<POSE_TYPE>::iteratePoses() {
  for (size_t robot : ordering_) {  // Iterate each optimizer once

    gtsam::Values neighbors = distributed_mappers_[robot]->neighbors();  // Get neighboring values

    // Ask other robots for updated estimates and update it
    for (const gtsam::Values::ConstKeyValuePair& key_value : neighbors) {
      gtsam::Key key = key_value.key;
      char symbol = gtsam::symbolChr(key);
      size_t neighboringRobotId = robot_names_.find(symbol);
      if (neighboringRobotId != std::string::npos) {  // if the robot is actually communicating
        // Update Neighbor Poses
        gtsam::Pose3 neighborPose = distributed_mappers_[neighboringRobotId]->currentEstimate().at<gtsam::Pose3>(key);
        distributed_mappers_[robot]->updateNeighbor(key, neighborPose);

        // Update Linearized Neighbor Pose
        gtsam::Vector poseEstimate = distributed_mappers_[neighboringRobotId]->linearizedPosesAt(key);
        distributed_mappers_[robot]->updateNeighborLinearizedPoses(key, poseEstimate);

        // Update the initialization status of the neighbor
        distributed_mappers_[robot]->updateNeighboringRobotInitialized(
            symbol, distributed_mappers_[neighboringRobotId]->isRobotInitialized());
      } else {
        // Robot we are not communicating with are considered as optimized
        distributed_mappers_[robot]->updateNeighboringRobotInitialized(symbol, true);
      }
    }

    // Iterate
    distributed_mappers_[robot]->estimatePoses();

    if (distributed_mappers_[robot]->updateType_ == distributed_mapper::DistributedMapper::incUpdate) {
      distributed_mappers_[robot]->updatePoses();
    }

    // This robot is initialized
    distributed_mappers_[robot]->updateInitialized(true);

    gtsam::VectorValues linearizedPoses = distributed_mappers_[robot]->linearizedPoses();
    gtsam::Values currentEstimate = distributed_mappers_[robot]->currentEstimate();
    current_estimates_[robot] =
        distributed_mapper::multirobot_util::retractPose3Global(currentEstimate, linearizedPoses);
  }

  // 2- change in the estimate of each robot is small enough
  bool converged = true;
  for (size_t robot = 0; robot < this->dataset_.robots().size(); robot++) {
    std::cout << "\t" << robot << ": " << distributed_mappers_[robot]->latestChange() << std::endl;
    converged = converged && distributed_mappers_[robot]->latestChange() <= 1e-2;  // "Tight" Bound from paper
  }
  return converged;
}

/*********************************************************************************************************************/
template <class POSE_TYPE>
BatchIterResults DGSRunner<POSE_TYPE>::iterate() {
  if (!converged_rotations_) {
    bool at_max_rotations = max_rotation_iters ? iter_count_ >= *max_rotation_iters : false;
    converged_rotations_ = at_max_rotations || iterateRotations();
  } else if (!initialized_poses_) {
    initialized_poses_ = true;
    for (size_t robot = 0; robot < this->dataset_.robots().size(); robot++) {
      distributed_mappers_[robot]->convertLinearizedRotationToPoses();
      current_estimates_[robot] = distributed_mappers_[robot]->currentEstimate();
      // reset initializations status for pose optimization
      distributed_mappers_[robot]->updateInitialized(false);
      distributed_mappers_[robot]->clearNeighboringRobotInit();
    }
  } else if (!converged_poses_) {
    bool at_max_poses = max_translation_iters ? iter_count_ >= (*max_translation_iters + *max_rotation_iters) : false;
    converged_poses_ = at_max_poses || iteratePoses();
  }

  // Aggregate the results
  std::vector<char> robots = this->dataset_.robots();
  jrl::Results result(this->dataset_.name(), this->name_, robots);
  for (size_t i = 0; i < robots.size(); i++) {
    auto types = this->dataset_.initializationWithTypes(robots[i]).types;
    result.robot_solutions[robots[i]] = jrl::TypedValues(current_estimates_[i], types);
  }

  // Augment the results
  // DGS returns solutions excluding other robot estimates, which is odd, but for now, just augment them in
  for (auto& rid : robots) {
    for (auto& kvp : result.robot_solutions[rid].types) {
      char oid = gtsam::Symbol(kvp.first).chr();
      if (oid != rid) {
        result.robot_solutions[rid].values.insert(kvp.first, result.robot_solutions[oid].values.at(kvp.first));
      }
    }
  }

  iter_count_++;
  return BatchIterResults(result, number_communication_edges_);
}
/*********************************************************************************************************************/
template <class POSE_TYPE>
bool DGSRunner<POSE_TYPE>::isConverged() {
  return converged_rotations_ && converged_poses_;
}

}  // namespace batch_runners