#pragma once
#include <DPGO/PGOAgent.h>
#include <jrl/Dataset.h>
#include <jrl/Results.h>

#include "BetweenChordalFactor.h"
#include "DistributedMapperUtils.h"
#include "MultiRobotUtils.h"
#include "batch_runners/BatchRunner.h"
#include "batch_runners/DGSRunner.h"

namespace batch_runners {
template <class POSE_TYPE>
class ASAPPRunner : public BatchRunner<POSE_TYPE> {
  /** Types **/
 public:
  struct Params {
    /** INITIALIZATION **/
    /// @brief The maximum number of iterations of rotation optimization we permit for DGS initialization
    size_t max_number_dgs_rotation_iters{50};  // Default from ASAPP paper
    /// @brief The maximum number of iterations of rotation optimization we permit for DGS initialization
    size_t max_number_dgs_pose_iters{50};  // Default from ASAPP paper

    /**  ASYNC LOOP **/
    /// @brief The probability that communication occurs between the active robot and one of its neighbors during any
    /// iteration. Note: each neighbor is sampled for independently
    double neighbor_communication_prob{0.5};

    /** OPTIMIZATION **/
    /// @brief the optimization method used by the underlying ROPT package
    DPGO::ROptParameters::ROptMethod optimization_method{DPGO::ROptParameters::ROptMethod::RGD};
    /// @brief Step size for gradient descent (ONLY USED IF METHOD=RGD)
    double rgd_step_size{0.05};  // Default Value from Parking Garage experiment in ASAPP paper
    /// @brief The number of iterations of RGD per active iteration (ONLY USED IF METHOD=RDG)
    size_t number_rgd_iters{10};
    /// @brief The rank of the relaxed SDP
    size_t relaxation_rank{5};  // Default 5 comes from ASAPP paper

    /** CONVERGENCE **/
    /// @brief The threshold for average update over the last #robot iters
    double norm_convergence_threshold{1e-4};
  };

  /// @brief Represents a dataset using DPGO Datatypes
  struct DPGODataset {
    std::vector<std::vector<DPGO::PriorSEMeasurement>> priors;
    std::vector<std::vector<DPGO::RelativeSEMeasurement>> odometry;
    std::vector<std::vector<DPGO::RelativeSEMeasurement>> private_loop_closures;
    std::vector<std::vector<DPGO::RelativeSEMeasurement>> public_loop_closures;
  };

  /** FIELDS **/
 protected:
  /// @brief Hyper parameters for ASAPP and our asynchronous implementation
  Params params_;
  /// @brief The pose dimension of the dataset saved here for easy access
  size_t pose_dim_;

  /// @brief Encapsulated DGSRunner used for computing initialization if configured
  std::shared_ptr<DGSRunner<POSE_TYPE>> dgs_runner_;
  /// @brief The last results generated from the DGS initialziation
  std::shared_ptr<BatchIterResults> dgs_results_;

  /// @brief The dataset represented in DPGO format (NOTE: this removes all priors)
  DPGODataset dpgo_dataset_;

  /// @brief The agents representing each robot in the algorithm
  std::vector<std::shared_ptr<DPGO::PGOAgent>> dpgo_agents_;
  /// @brief Flag indicating if we have initialized the DPGO agents with the output from DGS
  bool dpgo_initialized_{false};

  /// @brief Random Number Generator for sampling
  std::default_random_engine rng_;
  /// @brief Uniform distribution over the robots
  std::uniform_int_distribution<size_t> uniform_robot_distribution_;
  /// @brief Bernoulli distribution on the success of communication between a robot and its neighbors
  std::bernoulli_distribution bernoulli_communication_distribution_;

  /// @brief History of updates, we define convergence once the average over the last robots_.size() steps is less than
  /// thresh
  std::deque<double> norm_history_;
  /// @brief The previous iteration results (OF DPGO ONLY)
  std::shared_ptr<BatchIterResults> prev_iter_results;
  /// @brief Flag indicating whether or not the method has converged
  bool converged_{false};

  /** HELPERS **/
 protected:
  /// @brief Converts gtsam values into DPGO matrix
  /// Note: Filters out any poses owned by robots other than rid
  DPGO::Matrix convertValuesToMatrix(const char rid, const gtsam::Values& values) const;
  /// @brief Converts DPGO marix of poses to gtsam value format
  /// rid is the char identifier for the robot the matrix poses below to
  gtsam::Values convertMatrixToValues(const char rid, const DPGO::Matrix& matrix) const;

  /// @brief Initializes the specific agent parsing out its measurements from dataset_
  DPGODataset convertDatasetToDPGOFormat(jrl::Dataset dataset, DPGO::Matrix liftingMatrix) const;

  /// @brief Converts a noise model (INVARIANT: noise model is gaussian noise model) to (kappa, tau) the ASAPP params
  std::pair<double, double> convertGaussianCovarianceToKappaTau(const gtsam::SharedNoiseModel& noise_model) const;
  /// @brief Converts the provided factor (INVARIANT: factor is a Between<POSE_TYPE> factor) to RelativeSEMeasurement
  DPGO::RelativeSEMeasurement convertBetweenFactorToDPGOFormat(
      gtsam::NonlinearFactor::shared_ptr factor, const std::map<char, size_t>& robot_char_to_idx_map) const;
  /// @brief Converts the provided factor (INVARIANT: factor is a Prior<POSE_TYPE> factor) to PriorSEMeasurement
  DPGO::PriorSEMeasurement convertPriorFactorToDPGOFormat(gtsam::NonlinearFactor::shared_ptr factor,
                                                          const std::map<char, size_t>& robot_char_to_idx_map,
                                                          DPGO::Matrix liftingMatrix) const;

  /// @brief Preforms one iteration of the ASAPP algorithm
  BatchIterResults iterateDPGO(size_t selected_robot);
  /// @brief Simulates a communication between two DPGO agents
  void runDPGOCommunication(unsigned selected_robot_idx, unsigned other_robot_idx);
  /// @brief Convert current estimate from the DPGO agents into jrl format
  BatchIterResults getCurrentDPGOResults(size_t number_iter_comms);

  /// @brief Updates the history of norms and if converged sets flag converged_
  /// @param norm - the norm of variable changes for the selected robot
  void updateConvergence(double norm);

  /** Interface **/
 public:
  ASAPPRunner(std::string name, jrl::Dataset dataset, std::string output_dir, const Params params)
      : BatchRunner<POSE_TYPE>(name, dataset, output_dir), params_(params) {}
  BatchIterResults init() override;
  BatchIterResults iterate() override;
  bool isConverged() override;
  size_t iterationsPerFullComms() override;
};

}  // namespace batch_runners
#include "batch_runners/ASAPPRunner-inl.h"