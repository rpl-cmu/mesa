#pragma once
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

template <class POSE_TYPE>
class CorrectPrior : public gtsam::NoiseModelFactor1<POSE_TYPE> {
 protected:
  POSE_TYPE prior_;

 public:
  CorrectPrior(const gtsam::Key& key, const POSE_TYPE& prior, const gtsam::SharedNoiseModel& noiseModel)
      : gtsam::NoiseModelFactor1<POSE_TYPE>(noiseModel, key), prior_(prior) {}
  virtual gtsam::Vector evaluateError(const POSE_TYPE& val, boost::optional<gtsam::Matrix&> H = boost::none) const {
    return gtsam::traits<POSE_TYPE>::Local(this->prior_, val, boost::none, H);
  }
};
