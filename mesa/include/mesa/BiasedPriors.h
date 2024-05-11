#pragma once
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace biased_priors {

template <class POSE_TYPE>
class BiasedPrior : public gtsam::NoiseModelFactor1<POSE_TYPE> {
 protected:
  POSE_TYPE prior_;
  gtsam::Vector dual_;
  double beta_;

 public:
  BiasedPrior(const gtsam::Key& key, const POSE_TYPE& prior, const gtsam::SharedNoiseModel& noiseModel,
              const gtsam::Vector& dual, double beta)
      : gtsam::NoiseModelFactor1<POSE_TYPE>(noiseModel, key), prior_(prior), dual_(dual), beta_(beta) {}
  virtual gtsam::Vector evaluateError(const POSE_TYPE& val, boost::optional<gtsam::Matrix&> H = boost::none) const = 0;
};

/**
 *  ######   ########  #######  ########  ########  ######  ####  ######
 * ##    ##  ##       ##     ## ##     ## ##       ##    ##  ##  ##    ##
 * ##        ##       ##     ## ##     ## ##       ##        ##  ##
 * ##   #### ######   ##     ## ##     ## ######    ######   ##  ##
 * ##    ##  ##       ##     ## ##     ## ##             ##  ##  ##
 * ##    ##  ##       ##     ## ##     ## ##       ##    ##  ##  ##    ##
 *  ######   ########  #######  ########  ########  ######  ####  ######
 */
template <class POSE_TYPE>
class GeodesicBiasedPrior : public BiasedPrior<POSE_TYPE> {
 public:
  GeodesicBiasedPrior(const gtsam::Key& key, const POSE_TYPE& prior, const gtsam::SharedNoiseModel& noiseModel,
                      const gtsam::Vector& dual, double beta)
      : BiasedPrior<POSE_TYPE>(key, prior, noiseModel, dual, beta) {}

  gtsam::Vector evaluateError(const POSE_TYPE& val, boost::optional<gtsam::Matrix&> H = boost::none) const {
    gtsam::Vector err_vec = gtsam::traits<POSE_TYPE>::Local(this->prior_, val, boost::none, H);
    if (H) (*H) = sqrt(this->beta_) * (*H);
    return sqrt(this->beta_) * (err_vec + (this->dual_ / this->beta_));
  }
};

template <class POSE_TYPE>
class InverseGeodesicBiasedPrior : public BiasedPrior<POSE_TYPE> {
 public:
  InverseGeodesicBiasedPrior(const gtsam::Key& key, const POSE_TYPE& prior, const gtsam::SharedNoiseModel& noiseModel,
                             const gtsam::Vector& dual, double beta)
      : BiasedPrior<POSE_TYPE>(key, prior, noiseModel, dual, beta) {}

  gtsam::Vector evaluateError(const POSE_TYPE& val, boost::optional<gtsam::Matrix&> H = boost::none) const {
    gtsam::Vector err_vec = gtsam::traits<POSE_TYPE>::Local(val, this->prior_, H, boost::none);
    if (H) (*H) = sqrt(this->beta_) * (*H);
    return sqrt(this->beta_) * (err_vec + (this->dual_ / this->beta_));
  }
};

/**
 *    ###    ########  ##     ##  ######   ########  #######
 *   ## ##   ##     ##  ##   ##  ##    ##  ##       ##     ##
 *  ##   ##  ##     ##   ## ##   ##        ##       ##     ##
 * ##     ## ########     ###    ##   #### ######   ##     ##
 * ######### ##          ## ##   ##    ##  ##       ##     ##
 * ##     ## ##         ##   ##  ##    ##  ##       ##     ##
 * ##     ## ##        ##     ##  ######   ########  #######
 */
template <class POSE_TYPE>
class ApxGeoBiasedPrior : public BiasedPrior<POSE_TYPE> {
 public:
  ApxGeoBiasedPrior(const gtsam::Key& key, const POSE_TYPE& prior, const gtsam::SharedNoiseModel& noiseModel,
                    const gtsam::Vector& dual, double beta)
      : BiasedPrior<POSE_TYPE>(key, prior, noiseModel, dual, beta) {}

  gtsam::Vector evaluateError(const POSE_TYPE& val, boost::optional<gtsam::Matrix&> H = boost::none) const {
    gtsam::Vector err_vec = gtsam::traits<POSE_TYPE>::Logmap(val, H) - gtsam::traits<POSE_TYPE>::Logmap(this->prior_);
    if (H) (*H) = sqrt(this->beta_) * (*H);
    return sqrt(this->beta_) * (err_vec + (this->dual_ / this->beta_));
  }
};

template <class POSE_TYPE>
class InverseApxGeoBiasedPrior : public BiasedPrior<POSE_TYPE> {
 public:
  InverseApxGeoBiasedPrior(const gtsam::Key& key, const POSE_TYPE& prior, const gtsam::SharedNoiseModel& noiseModel,
                           const gtsam::Vector& dual, double beta)
      : BiasedPrior<POSE_TYPE>(key, prior, noiseModel, dual, beta) {}

  gtsam::Vector evaluateError(const POSE_TYPE& val, boost::optional<gtsam::Matrix&> H = boost::none) const {
    gtsam::Vector err_vec = gtsam::traits<POSE_TYPE>::Logmap(this->prior_) - gtsam::traits<POSE_TYPE>::Logmap(val, H);
    if (H) (*H) = sqrt(this->beta_) * -(*H);
    return sqrt(this->beta_) * (err_vec + (this->dual_ / this->beta_));
  }
};

/**
 *  ######  ########  ##       #### ########
 * ##    ## ##     ## ##        ##     ##
 * ##       ##     ## ##        ##     ##
 *  ######  ########  ##        ##     ##
 *       ## ##        ##        ##     ##
 * ##    ## ##        ##        ##     ##
 *  ######  ##        ######## ####    ##
 */
template <class POSE_TYPE>
class SplitBiasedPrior : public BiasedPrior<POSE_TYPE> {
 public:
  SplitBiasedPrior(const gtsam::Key& key, const POSE_TYPE& prior, const gtsam::SharedNoiseModel& noiseModel,
                   const gtsam::Vector& dual, double beta)
      : BiasedPrior<POSE_TYPE>(key, prior, noiseModel, dual, beta) {}

  gtsam::Vector evaluateError(const POSE_TYPE& val, boost::optional<gtsam::Matrix&> H = boost::none) const {
    // Translation Error
    gtsam::Matrix dtrans_dval, dlocal_dtrans;
    gtsam::Vector translation_error = gtsam::traits<typename POSE_TYPE::Translation>::Local(
        this->prior_.translation(), val.translation(&dtrans_dval), boost::none, &dlocal_dtrans);

    // Rotation Error
    gtsam::Matrix drot_dval, dlocal_drot;
    gtsam::Vector rotation_error = gtsam::traits<typename POSE_TYPE::Rotation>::Local(
        this->prior_.rotation(), val.rotation(&drot_dval), boost::none, &dlocal_drot);

    // Construct the result
    gtsam::Vector err_vec = gtsam::Vector::Zero(translation_error.size() + rotation_error.size());
    err_vec << translation_error, rotation_error;
    if (H) {
      gtsam::Matrix dtrans_err_dval = dlocal_dtrans * dtrans_dval;
      gtsam::Matrix drot_err_dval = dlocal_drot * drot_dval;

      // Vertical concatenation
      *H = gtsam::Matrix(dtrans_err_dval.rows() + drot_err_dval.rows(), dtrans_err_dval.cols());
      *H << dtrans_err_dval, drot_err_dval;
    }

    return sqrt(this->beta_) * (err_vec + (this->dual_ / this->beta_));
  }
};

template <class POSE_TYPE>
class InverseSplitBiasedPrior : public BiasedPrior<POSE_TYPE> {
 public:
  InverseSplitBiasedPrior(const gtsam::Key& key, const POSE_TYPE& prior, const gtsam::SharedNoiseModel& noiseModel,
                          const gtsam::Vector& dual, double beta)
      : BiasedPrior<POSE_TYPE>(key, prior, noiseModel, dual, beta) {}

  gtsam::Vector evaluateError(const POSE_TYPE& val, boost::optional<gtsam::Matrix&> H = boost::none) const {
    // Translation Error
    gtsam::Matrix dtrans_dval, dlocal_dtrans;
    gtsam::Vector translation_error = gtsam::traits<typename POSE_TYPE::Translation>::Local(
        val.translation(&dtrans_dval), this->prior_.translation(), &dlocal_dtrans, boost::none);

    // Rotation Error
    gtsam::Matrix drot_dval, dlocal_drot;
    gtsam::Vector rotation_error = gtsam::traits<typename POSE_TYPE::Rotation>::Local(
        val.rotation(&drot_dval), this->prior_.rotation(), &dlocal_drot, boost::none);

    // Construct the result
    gtsam::Vector err_vec = gtsam::Vector::Zero(translation_error.size() + rotation_error.size());
    err_vec << translation_error, rotation_error;
    if (H) {
      gtsam::Matrix dtrans_err_dval = dlocal_dtrans * dtrans_dval;
      gtsam::Matrix drot_err_dval = dlocal_drot * drot_dval;

      // Vertical concatenation
      *H = gtsam::Matrix(dtrans_err_dval.rows() + drot_err_dval.rows(), dtrans_err_dval.cols());
      *H << dtrans_err_dval, drot_err_dval;
    }

    return sqrt(this->beta_) * (err_vec + (this->dual_ / this->beta_));
  }
};

/**
 *  ######  ##     ##  #######  ########  ########     ###    ##
 * ##    ## ##     ## ##     ## ##     ## ##     ##   ## ##   ##
 * ##       ##     ## ##     ## ##     ## ##     ##  ##   ##  ##
 * ##       ######### ##     ## ########  ##     ## ##     ## ##
 * ##       ##     ## ##     ## ##   ##   ##     ## ######### ##
 * ##    ## ##     ## ##     ## ##    ##  ##     ## ##     ## ##
 *  ######  ##     ##  #######  ##     ## ########  ##     ## ########
 */
template <class POSE_TYPE>
class ChordalBiasedPrior : public gtsam::NoiseModelFactor1<POSE_TYPE> {
 protected:
  gtsam::Vector prior_;
  gtsam::Vector dual_;
  double beta_;

 public:
  ChordalBiasedPrior(const gtsam::Key& key, const gtsam::Vector& prior, const gtsam::SharedNoiseModel& noiseModel,
                     const gtsam::Vector& dual, double beta)
      : gtsam::NoiseModelFactor1<POSE_TYPE>(noiseModel, key), prior_(prior), dual_(dual), beta_(beta) {}

  gtsam::Vector evaluateError(const POSE_TYPE& val, boost::optional<gtsam::Matrix&> H = boost::none) const {
    gtsam::Vector err_vec = val.matrixElements(H) - this->prior_;
    return sqrt(this->beta_) * (err_vec + (this->dual_ / this->beta_));
  }
};

template <class POSE_TYPE>
class InverseChordalBiasedPrior : public gtsam::NoiseModelFactor1<gtsam::Vector> {
 protected:
  POSE_TYPE prior_;
  gtsam::Vector dual_;
  double beta_;

 public:
  InverseChordalBiasedPrior(const gtsam::Key& key, const POSE_TYPE& prior, const gtsam::SharedNoiseModel& noiseModel,
                            const gtsam::Vector& dual, double beta)
      : gtsam::NoiseModelFactor1<gtsam::Vector>(noiseModel, key), prior_(prior), dual_(dual), beta_(beta) {}

  gtsam::Vector evaluateError(const gtsam::Vector& val, boost::optional<gtsam::Matrix&> H = boost::none) const {
    gtsam::Vector err_vec = this->prior_.matrixElements() - val;
    if (H) {
      size_t d = err_vec.size();
      *H = -gtsam::Matrix::Identity(d, d);
    }
    return sqrt(this->beta_) * (err_vec + (this->dual_ / this->beta_));
  }
};
};  // namespace biased_priors