#pragma once
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include "mesa/BatchMESA.h"
#include "mesa/BiasedPriors.h"

/**
 * ##       #### ########
 * ##        ##  ##
 * ##        ##  ##
 * ##        ##  ######
 * ##        ##  ##
 * ##        ##  ##
 * ######## #### ########
 */
template <class POSE_TYPE>
class GeodesicMESA : public BatchMESA<POSE_TYPE, POSE_TYPE, biased_priors::GeodesicBiasedPrior<POSE_TYPE>,
                                      biased_priors::InverseGeodesicBiasedPrior<POSE_TYPE>> {
  /** Interface **/
 public:
  GeodesicMESA(std::vector<char> robots, std::map<char, gtsam::NonlinearFactorGraph> robot_graphs,
               std::map<char, gtsam::Values> robot_estimates, const MESAParams& params)
      : BatchMESA<POSE_TYPE, POSE_TYPE, biased_priors::GeodesicBiasedPrior<POSE_TYPE>,
                  biased_priors::InverseGeodesicBiasedPrior<POSE_TYPE>>(robots, robot_graphs, robot_estimates, params) {}
  /*********************************************************************************************************************/
  gtsam::Vector computeNewDualPose(const gtsam::Vector& dual, const double& beta, const POSE_TYPE& p,
                                   const POSE_TYPE& z) const override {
    return dual + beta * gtsam::traits<POSE_TYPE>::Local(z, p);
  }

  /*********************************************************************************************************************/
  gtsam::Vector computeNewDualShared(const gtsam::Vector& dual, const double& beta, const POSE_TYPE& p,
                                     const POSE_TYPE& z) const override {
    return computeNewDualPose(dual, beta, p, z);
  }

  /*********************************************************************************************************************/
  POSE_TYPE interpolateSPLIT(const POSE_TYPE& pa, const POSE_TYPE& pb, double alpha) const override {
    return baseInterpolateSPLIT(pa, pb, alpha);
  }

  /*********************************************************************************************************************/
  POSE_TYPE interpolateSLERP(const POSE_TYPE& pa, const POSE_TYPE& pb, double alpha) const override {
    return baseInterpolateSLERP(pa, pb, alpha);
  }

  size_t getDualDim() const override { return gtsam::traits<POSE_TYPE>::GetDimension(POSE_TYPE()); }
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
class ApxGeoMESA : public BatchMESA<POSE_TYPE, POSE_TYPE, biased_priors::ApxGeoBiasedPrior<POSE_TYPE>,
                                    biased_priors::InverseApxGeoBiasedPrior<POSE_TYPE>> {
  /** Interface **/
 public:
  ApxGeoMESA(std::vector<char> robots, std::map<char, gtsam::NonlinearFactorGraph> robot_graphs,
             std::map<char, gtsam::Values> robot_estimates, const MESAParams& params)
      : BatchMESA<POSE_TYPE, POSE_TYPE, biased_priors::ApxGeoBiasedPrior<POSE_TYPE>,
                  biased_priors::InverseApxGeoBiasedPrior<POSE_TYPE>>(robots, robot_graphs, robot_estimates, params) {
  }

  gtsam::Vector computeNewDualPose(const gtsam::Vector& dual, const double& beta, const POSE_TYPE& p,
                                   const POSE_TYPE& z) const override {
    return dual + beta * (gtsam::traits<POSE_TYPE>::Logmap(p) - gtsam::traits<POSE_TYPE>::Logmap(z));
  }

  /*********************************************************************************************************************/
  gtsam::Vector computeNewDualShared(const gtsam::Vector& dual, const double& beta, const POSE_TYPE& p,
                                     const POSE_TYPE& z) const override {
    return computeNewDualPose(dual, beta, p, z);
  }

  /*********************************************************************************************************************/
  POSE_TYPE interpolateSPLIT(const POSE_TYPE& pa, const POSE_TYPE& pb, double alpha) const override {
    return baseInterpolateSPLIT(pa, pb, alpha);
  }

  /*********************************************************************************************************************/
  POSE_TYPE interpolateSLERP(const POSE_TYPE& pa, const POSE_TYPE& pb, double alpha) const override {
    return baseInterpolateSLERP(pa, pb, alpha);
  }

  size_t getDualDim() const override { return gtsam::traits<POSE_TYPE>::GetDimension(POSE_TYPE()); }
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
class SplitMESA : public BatchMESA<POSE_TYPE, POSE_TYPE, biased_priors::SplitBiasedPrior<POSE_TYPE>,
                                   biased_priors::InverseSplitBiasedPrior<POSE_TYPE>> {
  /** Interface **/
 public:
  SplitMESA(std::vector<char> robots, std::map<char, gtsam::NonlinearFactorGraph> robot_graphs,
            std::map<char, gtsam::Values> robot_estimates, const MESAParams& params)
      : BatchMESA<POSE_TYPE, POSE_TYPE, biased_priors::SplitBiasedPrior<POSE_TYPE>,
                  biased_priors::InverseSplitBiasedPrior<POSE_TYPE>>(robots, robot_graphs, robot_estimates, params) {}

  gtsam::Vector computeNewDualPose(const gtsam::Vector& dual, const double& beta, const POSE_TYPE& p,
                                   const POSE_TYPE& z) const override {
    gtsam::Vector translation_error =
        gtsam::traits<typename POSE_TYPE::Translation>::Local(z.translation(), p.translation());
    gtsam::Vector rotation_error = gtsam::traits<typename POSE_TYPE::Rotation>::Local(z.rotation(), p.rotation());
    gtsam::Vector err_vec = gtsam::Vector::Zero(translation_error.size() + rotation_error.size());
    err_vec << translation_error, rotation_error;
    return dual + beta * err_vec;
  }

  /*********************************************************************************************************************/
  gtsam::Vector computeNewDualShared(const gtsam::Vector& dual, const double& beta, const POSE_TYPE& p,
                                     const POSE_TYPE& z) const override {
    return computeNewDualPose(dual, beta, p, z);
  }

  /*********************************************************************************************************************/
  POSE_TYPE interpolateSPLIT(const POSE_TYPE& pa, const POSE_TYPE& pb, double alpha) const override {
    return baseInterpolateSPLIT(pa, pb, alpha);
  }

  /*********************************************************************************************************************/
  POSE_TYPE interpolateSLERP(const POSE_TYPE& pa, const POSE_TYPE& pb, double alpha) const override {
    return baseInterpolateSLERP(pa, pb, alpha);
  }

  size_t getDualDim() const override { return gtsam::traits<POSE_TYPE>::GetDimension(POSE_TYPE()); }
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
class ChordalMESA : public BatchMESA<POSE_TYPE, gtsam::Vector, biased_priors::ChordalBiasedPrior<POSE_TYPE>,
                                     biased_priors::InverseChordalBiasedPrior<POSE_TYPE>> {
  /** Interface **/
 public:
  ChordalMESA(std::vector<char> robots, std::map<char, gtsam::NonlinearFactorGraph> robot_graphs,
              std::map<char, gtsam::Values> robot_estimates, const MESAParams& params)
      : BatchMESA<POSE_TYPE, gtsam::Vector, biased_priors::ChordalBiasedPrior<POSE_TYPE>,
                  biased_priors::InverseChordalBiasedPrior<POSE_TYPE>>(robots, robot_graphs, robot_estimates, params) {}

  gtsam::Vector computeNewDualPose(const gtsam::Vector& dual, const double& beta, const POSE_TYPE& p,
                                   const POSE_TYPE& z) const override {
    return dual + beta * (p.matrixElements() - z.matrixElements());
  }

  gtsam::Vector computeNewDualShared(const gtsam::Vector& dual, const double& beta, const POSE_TYPE& p,
                                     const gtsam::Vector& z) const override {
    return dual + beta * (p.matrixElements() - z);
  }

  std::pair<gtsam::Vector, gtsam::Matrix> computeNewZ(const POSE_TYPE& pa, const gtsam::Vector& dual_a,
                                                      const gtsam::Matrix& info_a, const POSE_TYPE& pb,
                                                      const gtsam::Vector& dual_b, const gtsam::Matrix& info_b,
                                                      const double beta) const {
    size_t d = dual_a.size();
    gtsam::Vector new_z = (pa.matrixElements() + pb.matrixElements()) / 2.0 + (dual_a + dual_b) / 2.0;
    return std::make_pair(new_z, gtsam::Matrix::Identity(d, d));
  }

  /*********************************************************************************************************************/
  gtsam::Vector interpolateSPLIT(const POSE_TYPE& pa, const POSE_TYPE& pb, double alpha) const override {
    return pa.matrixElements() + alpha * (-pa.matrixElements() + pb.matrixElements());
  }

  /*********************************************************************************************************************/
  gtsam::Vector interpolateSLERP(const POSE_TYPE& pa, const POSE_TYPE& pb, double alpha) const override {
    return pa.matrixElements() + alpha * (-pa.matrixElements() + pb.matrixElements());
  }

  size_t getDualDim() const override { return POSE_TYPE().matrixElements().size(); }
};