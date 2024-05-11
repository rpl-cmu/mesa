#pragma once
#include <jrl/Dataset.h>
#include <jrl/Results.h>

#include "BetweenChordalFactor.h"
#include "DistributedMapperUtils.h"
#include "MultiRobotUtils.h"
#include "batch_runners/BatchRunner.h"

namespace batch_runners {
template <class POSE_TYPE>
class DGSRunner : public BatchRunner<POSE_TYPE> {
  std::vector<boost::shared_ptr<distributed_mapper::DistributedMapper>> distributed_mappers_;
  std::vector<gtsam::Values> current_estimates_;
  bool converged_rotations_ = false;
  bool converged_poses_ = false;
  bool initialized_poses_ = false;
  std::vector<size_t> ordering_;
  std::string robot_names_;
  size_t iter_count_{0};
  size_t number_communication_edges_{0};
  /** Helpers **/
  bool iterateRotations();
  bool iteratePoses();

  /** Interface **/
 public:
  boost::optional<size_t> max_rotation_iters{boost::none};
  boost::optional<size_t> max_translation_iters{boost::none};

  DGSRunner(jrl::Dataset dataset, std::string output_dir) : BatchRunner<POSE_TYPE>("dgs", dataset, output_dir) {}
  BatchIterResults init() override;
  BatchIterResults iterate() override;
  bool isConverged() override;
};

}  // namespace batch_runners
#include "batch_runners/DGSRunner-inl.h"