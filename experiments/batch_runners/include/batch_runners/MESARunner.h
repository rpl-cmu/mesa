#pragma once
#include <jrl/Dataset.h>
#include <jrl/Results.h>

#include <iomanip>

#include "batch_runners/BatchRunner.h"
#include "mesa/BatchMESA.h"

namespace batch_runners {

template <class POSE_TYPE, class MESA_METHOD>
class BatchMESARunner : public BatchRunner<POSE_TYPE> {
 private:
  /// @brief The parameters for this specific MESA variant
  MESAParams params_;
  /// @brief The actual solver that we will use to iterate the solution
  std::shared_ptr<MESA_METHOD> solver_;

  /** ITERFACE **/
 public:
  BatchMESARunner(std::string name, jrl::Dataset dataset, std::string output_dir, MESAParams params)
      : BatchRunner<POSE_TYPE>(name, dataset, output_dir), params_(params) {}
  BatchIterResults init() override;
  BatchIterResults iterate() override;
  bool isConverged() override;
  size_t iterationsPerFullComms() override;
};

}  // namespace batch_runners
#include "batch_runners/MESARunner-inl.h"
