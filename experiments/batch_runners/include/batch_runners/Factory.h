#pragma once

#include <boost/shared_ptr.hpp>

#include "batch_runners/ASAPPRunner.h"
#include "batch_runners/Centralized.h"
#include "batch_runners/DDFSAM.h"
#include "batch_runners/DGSRunner.h"
#include "batch_runners/Independent.h"
#include "batch_runners/MBADMMRunner.h"
#include "batch_runners/MESARunner.h"
#include "mesa/BatchMESAVariants.h"

namespace batch_runners {

template <class POSE_TYPE>
gtsam::Vector compute_prior_sigmas() {
  if (std::is_same<POSE_TYPE, gtsam::Pose2>::value) {
    return (gtsam::Vector(3) << 1e2, 1e2, 2).finished();
  } else if (std::is_same<POSE_TYPE, gtsam::Pose3>::value) {
    return (gtsam::Vector(6) << 2, 2, 2, 1e2, 1e2, 1e2).finished();
  } else if (std::is_same<POSE_TYPE, gtsam::Point2>::value) {
    return (gtsam::Vector(2) << 1e2, 1e2).finished();
  } else if (std::is_same<POSE_TYPE, gtsam::Point3>::value) {
    return (gtsam::Vector(3) << 1e2, 1e2, 1e2).finished();
  } else {
    throw std::runtime_error("Could not determine the prior sigmas for given type");
  }
}

/**
 * ##    ##  #######  ##    ## ##       #### ##    ## ########    ###    ########
 * ###   ## ##     ## ###   ## ##        ##  ###   ## ##         ## ##   ##     ##
 * ####  ## ##     ## ####  ## ##        ##  ####  ## ##        ##   ##  ##     ##
 * ## ## ## ##     ## ## ## ## ##        ##  ## ## ## ######   ##     ## ########
 * ##  #### ##     ## ##  #### ##        ##  ##  #### ##       ######### ##   ##
 * ##   ### ##     ## ##   ### ##        ##  ##   ### ##       ##     ## ##    ##
 * ##    ##  #######  ##    ## ######## #### ##    ## ######## ##     ## ##     ##
 */
template <class POSE_TYPE>
boost::shared_ptr<batch_runners::BatchRunner<POSE_TYPE>> nonlinear_factory(std::string method_name,
                                                                           jrl::Dataset dataset,
                                                                           std::string output_dir) {
  std::cout << method_name << std::endl;
  std::cout << std::endl;
  if (method_name == "centralized") {
    return boost::make_shared<batch_runners::Centralized<POSE_TYPE>>(dataset, output_dir);
  }
  /*********************************************************************************************************************/
  else if (method_name == "independent") {
    return boost::make_shared<batch_runners::Independent<POSE_TYPE>>(dataset, output_dir);
  }
  /*********************************************************************************************************************/
  else if (method_name == "ddfsam") {
    return boost::make_shared<batch_runners::DDFSAM<POSE_TYPE>>(dataset, output_dir);
  }
  /*********************************************************************************************************************/
  else if (method_name == "asapp") {
    typename batch_runners::ASAPPRunner<POSE_TYPE>::Params params;
    params.max_number_dgs_rotation_iters = 50;
    params.max_number_dgs_pose_iters = 50;
    params.rgd_step_size = 1e-3;
    params.number_rgd_iters = 100;
    params.optimization_method = DPGO::ROptParameters::ROptMethod::RGD;
    return boost::make_shared<batch_runners::ASAPPRunner<POSE_TYPE>>("asapp", dataset, output_dir, params);
  }
  /*********************************************************************************************************************/
  else if (method_name == "dgs") {
    // WARNING ONLY WORKS WITH POSE 3
    return boost::make_shared<batch_runners::DGSRunner<POSE_TYPE>>(dataset, output_dir);
  }
  /*********************************************************************************************************************/
  else if (method_name == "mbadmm") {
    return boost::make_shared<batch_runners::MBADMMRunner<POSE_TYPE>>("mbadmm", dataset, output_dir, true, false);
  }
  /*********************************************************************************************************************/
  else if (method_name == "geodesic-mesa") {
    MESAParams params;
    params.beta_init = 1;
    params.beta_multiplier_increase = 1.0;
    params.prior_shared_vars_on_indep_solve = true;
    params.shared_var_prior_sigmas = compute_prior_sigmas<POSE_TYPE>();

    // Variant Specific PARMS
    params.z_compute_method = MESAParams::ComputeZMethod::INTERPOLATE_SPLIT;
    params.weight_z_compute = false;
    params.dual_compute_target = MESAParams::DualComputeTarget::OTHER_ESTIMATE;

    return boost::make_shared<batch_runners::BatchMESARunner<POSE_TYPE, GeodesicMESA<POSE_TYPE>>>(
        "geodesic-mesa", dataset, output_dir, params);
  }
  /*********************************************************************************************************************/
  else if (method_name == "split-mesa") {
    MESAParams params;
    params.beta_init = 200;
    params.beta_multiplier_increase = 1.05;
    params.prior_shared_vars_on_indep_solve = true;
    params.shared_var_prior_sigmas = compute_prior_sigmas<POSE_TYPE>();

    // Variant Specific PARMS
    params.z_compute_method = MESAParams::ComputeZMethod::INTERPOLATE_SPLIT;
    params.weight_z_compute = false;
    params.dual_compute_target = MESAParams::DualComputeTarget::OTHER_ESTIMATE;

    return boost::make_shared<batch_runners::BatchMESARunner<POSE_TYPE, SplitMESA<POSE_TYPE>>>("split-mesa", dataset,
                                                                                                 output_dir, params);
  }
  /*********************************************************************************************************************/
  else if (method_name == "chordal-mesa") {
    MESAParams params;
    params.beta_init = 200;
    params.beta_multiplier_increase = 1.0;
    params.prior_shared_vars_on_indep_solve = true;
    params.shared_var_prior_sigmas = compute_prior_sigmas<POSE_TYPE>();

    // Variant Specific PARMS
    params.weight_z_compute = false;
    params.dual_compute_target = MESAParams::DualComputeTarget::OTHER_ESTIMATE;

    return boost::make_shared<batch_runners::BatchMESARunner<POSE_TYPE, ChordalMESA<POSE_TYPE>>>(
        "chordal-mesa", dataset, output_dir, params);
  }
  /*********************************************************************************************************************/
  else if (method_name == "apxgeo-mesa") {
    MESAParams params;
    params.beta_init = 200;
    params.beta_multiplier_increase = 1.0;
    params.prior_shared_vars_on_indep_solve = true;
    params.shared_var_prior_sigmas = compute_prior_sigmas<POSE_TYPE>();

    // Variant Specific PARMS
    params.weight_z_compute = false;
    params.dual_compute_target = MESAParams::DualComputeTarget::OTHER_ESTIMATE;

    return boost::make_shared<batch_runners::BatchMESARunner<POSE_TYPE, ApxGeoMESA<POSE_TYPE>>>(
        "apxgeo-mesa", dataset, output_dir, params);
  }
  /*********************************************************************************************************************/
  else {
    throw std::runtime_error("Unknown Nonlinear Batch Method Name");
  }
}

/**
 * ##       #### ##    ## ########    ###    ########
 * ##        ##  ###   ## ##         ## ##   ##     ##
 * ##        ##  ####  ## ##        ##   ##  ##     ##
 * ##        ##  ## ## ## ######   ##     ## ########
 * ##        ##  ##  #### ##       ######### ##   ##
 * ##        ##  ##   ### ##       ##     ## ##    ##
 * ######## #### ##    ## ######## ##     ## ##     ##
 */
template <class POSE_TYPE>
boost::shared_ptr<batch_runners::BatchRunner<POSE_TYPE>> linear_factory(std::string method_name, jrl::Dataset dataset,
                                                                        std::string output_dir) {
  if (method_name == "centralized") {
    return boost::make_shared<batch_runners::Centralized<POSE_TYPE>>(dataset, output_dir);
  }
  /*********************************************************************************************************************/
  else if (method_name == "independent") {
    return boost::make_shared<batch_runners::Independent<POSE_TYPE>>(dataset, output_dir);
  }
  /*********************************************************************************************************************/
  else if (method_name == "ddfsam") {
    return boost::make_shared<batch_runners::DDFSAM<POSE_TYPE>>(dataset, output_dir);
  }
  /*********************************************************************************************************************/
  else if (method_name == "geodesic-mesa") {
    MESAParams params;
    params.beta_init = 200;
    params.beta_multiplier_increase = 1.00;
    params.prior_shared_vars_on_indep_solve = true;
    params.shared_var_prior_sigmas = compute_prior_sigmas<POSE_TYPE>();

    // Variant Specific PARMS
    params.z_compute_method = MESAParams::ComputeZMethod::INTERPOLATE_SPLIT;
    params.weight_z_compute = false;
    params.dual_compute_target = MESAParams::DualComputeTarget::OTHER_ESTIMATE;

    return boost::make_shared<batch_runners::BatchMESARunner<POSE_TYPE, GeodesicMESA<POSE_TYPE>>>(
        "geodesic-mesa", dataset, output_dir, params);
  }
  /*********************************************************************************************************************/
  else {
    throw std::runtime_error("Unknown linear Batch Method Name");
  }
}
}  // namespace batch_runners
