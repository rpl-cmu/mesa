#pragma once
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <random>

/** TYPES **/
/// @brief Map[Owner Robot -> Map[Shared With Robot -> Key]]
typedef std::map<char, std::map<char, std::set<gtsam::Key>>> RobotSharedVariables;
/// @brief Edge in the communication network, INVARIANT: first < second
typedef std::pair<char, char> CommunicationEdge;
/// @brief Map[Owner Robot -> Local Factor Graph]
typedef std::map<char, gtsam::NonlinearFactorGraph> RobotGraphs;
/// @brief Map[Owner Robot -> Current Estimate to Local Graph]
typedef std::map<char, gtsam::Values> RobotValues;

/// @brief Parameters for the MESA Method
struct MESAParams {
  /// @brief The threshold for determining convergence
  double convergence_threshold{1e-4};
  /// @brief The initial beta value
  double beta_init{10.0};
  /// @brief The rate of increase for beta
  double beta_multiplier_increase{1.0};

  /// @brief Flag indicating pseudo-syncing of beta update (update ever |R| steps)
  bool psuedo_sync_beta{false};

  /// @brief Flag indicating to add weak priors to shared variables in indep solve. Used when inter-robot measurements
  /// are not full determined
  bool prior_shared_vars_on_indep_solve{false};
  gtsam::Vector shared_var_prior_sigmas;

  enum ComputeZMethod {
    OPTIMIZE,           // Solve the optimization problem defined by ADMM
    INTERPOLATE_SPLIT,  // Approx the opt with SPLIT interpolation (TRUE WHEN DUAL == 0)
    INTERPOLATE_SLERP,  // Approx the opt with SLERP interpolation (not valid, but useful)
  };
  /// @brief Approximate the z optimization
  ComputeZMethod z_compute_method{ComputeZMethod::OPTIMIZE};
  /// @brief Compute Z using weights from variable information estimates
  bool weight_z_compute{false};

  enum DualComputeTarget {
    SHARED_ESTIMATE,   // The consistent shared estimate agreed on by both robots
    OTHER_ESTIMATE,    // The current estimate of the other robot
    UNWEIGHTED_SLERP,  // A Pose at SLERP(0.5) between the two robots (dual_a = - dual_b)
    CHOUDHARY_TARGET,  // Other estimate for robot a, but is incorrectly negated for robot b
  };
  DualComputeTarget dual_compute_target{DualComputeTarget::SHARED_ESTIMATE};

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  //  needed for eigen member
};

template <class POSE_TYPE, class Z_TYPE, class BIASED_PRIOR_TYPE, class INV_BIASED_PRIOR_TYPE>
class BatchMESA {
  /** FIELDS **/
 protected:
  MESAParams params_;

  /// @brief The robots involved in the problem
  std::vector<char> robots_;
  /// @brief The base graphs for each robot
  RobotGraphs robot_base_graphs_;

  /// @brief Graphs with potentially very week priors on shared variables to ensure a determined system
  RobotGraphs robot_determined_graphs_;
  /// @brief The base marginals
  std::map<char, gtsam::Marginals> robot_base_marginals_;

  /// @brief The current estimate for each individual robot
  RobotValues robot_estimates_;

  /// @brief Each robot's shared variables
  RobotSharedVariables robot_shared_variables_;
  /// @brief The consistent estimates for all shared variables notice a shared variable may appear in multiple edges
  std::map<CommunicationEdge, gtsam::Values> edge_shared_estimates_;
  std::map<CommunicationEdge, std::map<gtsam::Key, gtsam::SharedNoiseModel>> edge_shared_noise_models_;
  /// @brief Each robot's dual variables
  std::map<CommunicationEdge, std::map<char, gtsam::Values>> edge_robot_dual_variables_;
  /// @brief The Lie-Formulation dual variables, maintained for computing The Dual Residual
  std::map<CommunicationEdge, std::map<char, gtsam::Values>> edge_robot_lie_dual_variables_;

  /// @brief The edges that exist in the current communication network as defined implicitly by the problem
  std::set<CommunicationEdge> communication_network_;
  /// @brief The current value of beta, one for each neighbor
  std::map<CommunicationEdge, double> edge_beta_variables_;
  /// @brief Number of communications along each communication edge
  std::map<char, std::map<CommunicationEdge, size_t>> robot_counts_since_last_comm_;

  /// @brief Random Device for generating random numbers to sample communication edges
  std::mt19937 random_number_generator_;
  std::uniform_int_distribution<int> communication_edge_distribution_;

  /// @brief History of updates, we define convergence once the average over the last robots_.size() steps is less than
  /// thresh
  std::deque<double> norm_history_;

  /// @brief Flag tracking whether the method has converged yet
  bool converged_{false};
  /// @brief Flag tracking the first call to iterate which has different behavior than a typical iterate
  bool first_iterate_{true};

  /** Helpers **/
 protected:
  /// @brief Determines the shared variables between robots given their local factor-graphs
  static RobotSharedVariables parseSharedVariables(std::vector<char> robots, RobotGraphs robot_graphs);

  /// @brief Determines the communication network given the variables shared between robots
  static std::set<CommunicationEdge> parseCommunicationNetwork(std::vector<char> robots,
                                                               RobotSharedVariables robot_shared_variables);

  /// @brief Computes a graph where shared variables have an additional prior to account for inter-robot measurements
  /// that do no fully determine robot poses.
  static RobotGraphs constructDeterminedGraphs(std::vector<char> robots, RobotGraphs robot_base_graphs,
                                               RobotValues robot_estimates, RobotSharedVariables robot_shared_variables,
                                               bool prior_shared_variables, gtsam::Vector shared_prior_sigma);

  /// @brief x^t+1 Step Updates the x variables for the given robot
  void updateRobotEstimateAndMarginals(char rid);

  /// @brief z^t+1 Step: Updates the shared variable estimate and noise model for all variables shared between the two
  /// given robots
  void updateSharedVariables(CommunicationEdge robots);

  /// @brief \lambda^t+1 Step: Updates the dual variables for all shared variables between the two given robots
  void updateDualVariables(CommunicationEdge robots);

  /// @brief Updates the the counts since last comm for the given robot given its communicating over the given edge
  /// Returns true iff the given robot has had sufficient communications since the last one along this edge to update
  /// beta
  bool updateCountsSinceLastComm(CommunicationEdge current_edge);

  /// @brief  Runs all the steps during communication between two robots
  /// @param comm_edge
  void preformCommunicationStep(CommunicationEdge comm_edge);

  /// @brief Checks convergence criteria and updates flag is converged
  void updateConvergence(double step_norm);

  /** Interface **/
 public:
  BatchMESA(std::vector<char> robots, RobotGraphs robot_graphs, RobotValues robot_estimates, MESAParams params);
  /// @brief Setup code
  std::map<char, gtsam::Values> init();

  /// @brief Runs a single step of the algorithm assuming two random robots are communicating
  /// @return Estimates after the current iteration
  std::map<char, gtsam::Values> iterate();

  /// @brief returns true iff the method has converged
  bool isConverged() { return converged_; }

  /// @brief returns the number of edges in the communication network.
  /// In expectation this is the number of iterations we need to globally decrease the cost function
  size_t commNetworkSize() { return communication_network_.size(); }

  /// @brief returns the dimension of the dual variable
  virtual size_t getDualDim() const = 0;

 protected:
  /// @brief Computes an updated edge variable
  virtual std::pair<Z_TYPE, gtsam::Matrix> computeNewZ(const POSE_TYPE& pa, const gtsam::Vector& dual_a,
                                                       const gtsam::Matrix& info_a, const POSE_TYPE& pb,
                                                       const gtsam::Vector& dual_b, const gtsam::Matrix& info_b,
                                                       const double beta) const;
  /// @brief Computes an updated dual variable for a vector type edge variable Z
  virtual gtsam::Vector computeNewDualShared(const gtsam::Vector& dual, const double& beta, const POSE_TYPE& p,
                                             const Z_TYPE& z) const = 0;
  /// @brief Computes an updated dual variable for a pose type edge variable Z 
  virtual gtsam::Vector computeNewDualPose(const gtsam::Vector& dual, const double& beta, const POSE_TYPE& p,
                                           const POSE_TYPE& z) const = 0;

  /// @brief Interpolates two poses using linear interp for translation and slerp for rotation
  virtual Z_TYPE interpolateSPLIT(const POSE_TYPE& pa, const POSE_TYPE& pb, double alpha) const = 0;
  /// @brief Interpolates two poses using slerp
  virtual Z_TYPE interpolateSLERP(const POSE_TYPE& pa, const POSE_TYPE& pb, double alpha) const = 0;
};

#include "mesa/BatchMESA-inl.h"