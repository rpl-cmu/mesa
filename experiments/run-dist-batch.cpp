#include <jrl/Dataset.h>
#include <jrl/Parser.h>
#include <jrl/Writer.h>

#include <boost/program_options.hpp>
#include "batch_runners/Factory.h"

/**
 *    ###    ########   ######    ######
 *   ## ##   ##     ## ##    ##  ##    ##
 *  ##   ##  ##     ## ##        ##
 * ##     ## ########  ##   ####  ######
 * ######### ##   ##   ##    ##        ##
 * ##     ## ##    ##  ##    ##  ##    ##
 * ##     ## ##     ##  ######    ######
 */

namespace po = boost::program_options;
po::variables_map handle_args(int argc, const char *argv[]) {
  // Define the options
  po::options_description options("Allowed options");
  // clang-format off
  options.add_options()
      ("help,h",                                                      "Produce help message")
      ("input_jrl_file,i",    po::value<std::string>()->required(),   "(Required) Path to Incremental Robot Log file.")
      ("method,m",            po::value<std::string>()->required(),   "(Required) The name of the method to run see batch_runners/include/Factory.h for more options.")
      ("output_dir,o",        po::value<std::string>()->required(),   "(Required) Directory to which the results will be saved.")
      ("is3d",                po::bool_switch(),                      "Flag indicating the dataset is 3D (Pose3 / Point3).")
      ("linear",              po::bool_switch(),                      "Flag indicating the dataset is linear (Point2 / Point3)");
  // clang-format on

  // Parse and return the options
  po::variables_map var_map;
  po::store(po::parse_command_line(argc, argv, options), var_map);

  // Handle help special case
  if (var_map.count("help") || argc == 1) {
    std::cout << "run-experiment: Main entry point to run distributed SLAM methods on JRL datasets. Please provide "
                 "required arguments: "
              << std::endl;
    std::cout << options << "\n";
    exit(1);
  }

  // Handle all other arguments
  po::notify(var_map);

  return var_map;
}

/**
 * ##     ##    ###    #### ##    ##
 * ###   ###   ## ##    ##  ###   ##
 * #### ####  ##   ##   ##  ####  ##
 * ## ### ## ##     ##  ##  ## ## ##
 * ##     ## #########  ##  ##  ####
 * ##     ## ##     ##  ##  ##   ###
 * ##     ## ##     ## #### ##    ##
 */
int main(int argc, const char *argv[]) {
  auto args = handle_args(argc, argv);
  jrl::Parser parser;
  jrl::Dataset dataset = parser.parseDataset(args["input_jrl_file"].as<std::string>());

  std::cout << "Running " << args["method"].as<std::string>() << " on ..." << std::endl;
  std::cout << "Dataset" << std::endl;
  std::cout << "----------------------------------" << std::endl;
  std::cout << dataset.name() << std::endl;
  std::cout << "----------------------------------" << std::endl;

  std::cout << "Status: Building Method Runner" << std::endl;

  if (args["is3d"].as<bool>()) {
    if (args["linear"].as<bool>()) {
      boost::shared_ptr<batch_runners::BatchRunner<gtsam::Point3>> method =
          batch_runners::linear_factory<gtsam::Point3>(args["method"].as<std::string>(), dataset,
                                                       args["output_dir"].as<std::string>());
      std::cout << "Status: Starting" << std::endl;
      method->optimize();
    } else {
      boost::shared_ptr<batch_runners::BatchRunner<gtsam::Pose3>> method =
          batch_runners::nonlinear_factory<gtsam::Pose3>(args["method"].as<std::string>(), dataset,
                                                         args["output_dir"].as<std::string>());
      std::cout << "Status: Starting" << std::endl;
      method->optimize();
    }
  } else {
    if (args["linear"].as<bool>()) {
      boost::shared_ptr<batch_runners::BatchRunner<gtsam::Point2>> method =
          batch_runners::linear_factory<gtsam::Point2>(args["method"].as<std::string>(), dataset,
                                                       args["output_dir"].as<std::string>());
      std::cout << "Status: Starting" << std::endl;
      method->optimize();
    } else {
      boost::shared_ptr<batch_runners::BatchRunner<gtsam::Pose2>> method =
          batch_runners::nonlinear_factory<gtsam::Pose2>(args["method"].as<std::string>(), dataset,
                                                         args["output_dir"].as<std::string>());
      std::cout << "Status: Starting" << std::endl;
      method->optimize();
    }
  }

  std::cout << "Status: Done" << std::endl;
  return 0;
}