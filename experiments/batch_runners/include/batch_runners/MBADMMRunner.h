#pragma once
#include "ADMM.h"
#include "ADMMUtils.h"
#include <jrl/Dataset.h>
#include <jrl/Results.h>

#include "batch_runners/BatchRunner.h"

namespace batch_runners {
template <class POSE_TYPE>
class MBADMMRunner : public BatchRunner<POSE_TYPE> {
  std::shared_ptr<ADMM<POSE_TYPE>> mbadmm_solver_;
  double p_res_{0.0};
  double d_res_{0.0};
  size_t iter_count_{0};
  bool converged_{false};
  bool parallel_admm_{false};
  bool adaptive_penalty_{true};
  size_t number_communication_edges_{0};

  /** Interface **/
 public:
  MBADMMRunner(std::string name, jrl::Dataset dataset, std::string output_dir, bool adaptive_penalty,
             bool parallel_admm)
      : BatchRunner<POSE_TYPE>(name, dataset, output_dir),
        adaptive_penalty_(adaptive_penalty),
        parallel_admm_(parallel_admm) {}
  BatchIterResults init() override;
  BatchIterResults iterate() override;
  bool isConverged() override;
};

}  // namespace batch_runners
#include "batch_runners/MBADMMRunner-inl.h"