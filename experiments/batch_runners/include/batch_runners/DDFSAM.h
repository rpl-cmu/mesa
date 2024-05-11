#pragma once
#include <jrl/Dataset.h>
#include <jrl/Results.h>

#include "batch_runners/BatchRunner.h"

namespace batch_runners {
template <class POSE_TYPE>
class DDFSAM : public BatchRunner<POSE_TYPE> {
  /** Interface **/
 public:
  DDFSAM(jrl::Dataset dataset, std::string output_dir)
      : BatchRunner<POSE_TYPE>("ddfsam", dataset, output_dir) {}
  BatchIterResults init() override;
  BatchIterResults iterate() override;
  bool isConverged() override;
};

}  // namespace batch_runners
#include "batch_runners/DDFSAM-inl.h"