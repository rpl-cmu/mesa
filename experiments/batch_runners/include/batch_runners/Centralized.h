#pragma once
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <jrl/Dataset.h>
#include <jrl/Results.h>

#include "batch_runners/BatchRunner.h"

namespace batch_runners {

template<class POSE_TYPE>
class Centralized : public BatchRunner<POSE_TYPE> {
  /** Fields **/
 protected:
  /// @brief The joint factor graph of the runner's dataset
  gtsam::NonlinearFactorGraph joint_factor_graph_;
  /// @brief The joint values of the runner's dataset
  gtsam::Values joint_values_;

  /** Interface **/
 public:
  Centralized(jrl::Dataset dataset, std::string output_dir)
      : BatchRunner<POSE_TYPE>("centralized", dataset, output_dir) {}
  BatchIterResults init() override;
  BatchIterResults iterate() override;
  bool isConverged() override;
};

}  // namespace batch_runners

#include "batch_runners/Centralized-inl.h"