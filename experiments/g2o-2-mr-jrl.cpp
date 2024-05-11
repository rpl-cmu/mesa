
#include <ADMM.h>
#include <ADMMUtils.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/dataset.h>
#include <jrl/DatasetBuilder.h>
#include <jrl/IOMeasurements.h>
#include <jrl/IOValues.h>
#include <jrl/Writer.h>

#include <boost/program_options.hpp>

std::vector<char> ROBOT_ID_OPTIONS = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm',
                                      'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z'};

namespace po = boost::program_options;
po::variables_map handle_args(int argc, const char* argv[]) {
  // Define the options
  po::options_description options("Allowed options");
  // clang-format off
  options.add_options()
      ("help,h",                                                      "Produce help message")
      ("input_g2o,i",         po::value<std::string>()->required(),   "(Required) The input g2o file.")
      ("name,n",              po::value<std::string>()->required(),   "(Required) The name of the dataset")
      ("output_jrl,o",        po::value<std::string>()->required(),   "(Required) The output jrl file.")
      ("num_partitions,p",    po::value<int>()->required(),           "(Required) The number of partitions to make.");
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

int main(int argc, const char* argv[]) {
  auto args = handle_args(argc, argv);
  int num_partitions = args["num_partitions"].as<int>();

  // Read the g2o File
  gtsam::GraphAndValues readGraph = gtsam::readG2o(args["input_g2o"].as<std::string>(), true);
  gtsam::NonlinearFactorGraph graph = *(readGraph.first);
  gtsam::Values initial = *(readGraph.second);

  // Partition the given graph into subgraphs using metis
  // Returns map from variable key -> subgraph index
  std::map<int, int> variable_partition = metis(graph, num_partitions);

  // Get a sorted list of the variables held by each robot
  std::map<char, std::vector<int>> robot_variables;
  for (int i = 0; i < num_partitions; i++) robot_variables[ROBOT_ID_OPTIONS[i]] = std::vector<int>();
  for (auto& kvp : variable_partition) robot_variables[ROBOT_ID_OPTIONS[kvp.second]].push_back(kvp.first);
  for (int i = 0; i < num_partitions; i++)
    std::sort(robot_variables[ROBOT_ID_OPTIONS[i]].begin(), robot_variables[ROBOT_ID_OPTIONS[i]].end());

  // Construct a global mapping to jrl keys renumbering each partition to work with ASAPP
  std::map<gtsam::Key, gtsam::Key> variable_remapping;
  for (auto& kvp : robot_variables) {
    char rid = kvp.first;
    std::vector<int> g2o_var_keys = kvp.second;

    for (int i = 0; i < g2o_var_keys.size(); i++) {
      variable_remapping[g2o_var_keys[i]] = gtsam::symbol(rid, i);
    }
  }

  // Remap the Graph
  gtsam::NonlinearFactorGraph rekeyed_graph = graph.rekey(variable_remapping);

  // Remap the initialization
  gtsam::Values rekeyed_initial;
  for (const auto& kvp : initial) {
    gtsam::Key new_key = variable_remapping[kvp.key];
    rekeyed_initial.insert(new_key, kvp.value);
  }

  // Add a prior since g2o doesn't include one 
  rekeyed_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      variable_remapping[0], rekeyed_initial.at<gtsam::Pose3>(variable_remapping[0]),
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.5, 0.5, 0.5, 1, 1, 1).finished()));


  // Solve the G2O file using GTSAM to get pseudo GT
  gtsam::LevenbergMarquardtParams params;
  params.setErrorTol(1e-8);
  params.setMaxIterations(1000);
  gtsam::LevenbergMarquardtOptimizer optimizer(rekeyed_graph, rekeyed_initial, params);
  gtsam::Values rekeyed_pseudo_gt = optimizer.optimize();

  // For each robot (except the first) add a prior
  for (int i = 1; i < num_partitions; i++) {
    size_t rid_first_var_idx = robot_variables[ROBOT_ID_OPTIONS[i]].front();
    gtsam::Key rid_first_var_key = variable_remapping[rid_first_var_idx];
    //rekeyed_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
    //    rid_first_var_key, rekeyed_pseudo_gt.at<gtsam::Pose3>(rid_first_var_key),
    //    gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1).finished()));
  }

  // Build the JRL dataset
  std::vector<char> robots(ROBOT_ID_OPTIONS.begin(), ROBOT_ID_OPTIONS.begin() + num_partitions);
  jrl::DatasetBuilder builder(args["name"].as<std::string>(), robots);

  // Separate each robots information
  for (int i = 0; i < num_partitions; i++) {
    char rid = ROBOT_ID_OPTIONS[i];
    gtsam::NonlinearFactorGraph robot_graph;
    std::vector<std::string> robot_factor_types;
    gtsam::Values robot_initial;
    gtsam::Values robot_pseudo_gt;
    jrl::ValueTypes robot_value_types;

    for (auto& factor : rekeyed_graph) {
      auto keys = factor->keys();
      std::vector<gtsam::Symbol> symbols;
      for (auto key : keys) {
        std::cout << gtsam::DefaultKeyFormatter(key) << " ";
        symbols.push_back(gtsam::Symbol(key));
      }
      std::cout << std::endl;
      // A factor gets assigned to the robot iff it owns all variables, or it owns the first variable
      bool rid_owns_factor = (keys.size() == 1 && symbols.front().chr() == rid) ||
                             (keys.size() == 2 && symbols.front().chr() == rid && symbols.back().chr() == rid) ||
                             (keys.size() == 2 && symbols.front().chr() == rid && symbols.back().chr() != rid);

      std::cout << (keys.size() == 2 && symbols.front().chr() == rid && symbols.back().chr() != rid) << std::endl;
      // If the robot owns this factor
      if (rid_owns_factor) {
        std::cout << "Adding Factor" << std::endl;
        // Add the factor to the robots graph
        robot_graph.push_back(factor);
        robot_factor_types.push_back(keys.size() == 1 ? jrl::PriorFactorPose3Tag : jrl::BetweenFactorPose3Tag);

        // Add the initial and gt variables to for the robot
        for (auto key : keys) {
          if (!robot_initial.exists(key)) {
            robot_initial.insert(key, rekeyed_initial.at(key));
            robot_pseudo_gt.insert(key, rekeyed_pseudo_gt.at(key));
            robot_value_types[key] = jrl::Pose3Tag;
          }
        }
      }
      std::cout << std::endl;
    }

    // Add the robot info to the dataset
    jrl::TypedValues typed_initial = jrl::TypedValues(robot_initial, robot_value_types);
    jrl::TypedValues typed_pseudo_gt = jrl::TypedValues(robot_pseudo_gt, robot_value_types);
    builder.addEntry(robots[i], 0, robot_graph, robot_factor_types, {}, typed_initial, typed_pseudo_gt);

    robot_graph.saveGraph(std::string(1, rid) + ".dot");
  }

  jrl::Dataset dataset = builder.build();
  jrl::Writer writer;
  writer.writeDataset(dataset, args["output_jrl"].as<std::string>());
}
