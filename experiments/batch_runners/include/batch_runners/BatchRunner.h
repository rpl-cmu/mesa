/** @brief Provides interface used to run experiments.
 *
 * @author Dan McGann
 * @date Nov 2022
 */
#pragma once
#include <jrl/Dataset.h>
#include <jrl/Metrics.h>
#include <jrl/Results.h>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <ctime>
#include <iomanip>

namespace bfs = boost::filesystem;
namespace batch_runners {

struct BatchIterResults {
  /// @brief The results from an iteration of the distributed algorithm
  jrl::Results results;
  /// @brief  The number of communications that occurred in this iteration
  size_t number_communications;
  BatchIterResults(jrl::Results results, size_t number_communications)
      : results(results), number_communications(number_communications) {}
  BatchIterResults(BatchIterResults& other)
      : results(other.results), number_communications(other.number_communications) {}
};

/// @brief Interface that each method must provide to run the experiments
template <class POSE_TYPE>
class BatchRunner {
 protected:
  /// @brief The name of the underlying method
  std::string name_;
  /// @brief The dataset we are running this method on
  jrl::Dataset dataset_;

 public:
  /// @brief The directory in which to save results
  std::string output_directory_;
  /// @brief The directory in which to save intermediate iteration results
  std::string iteration_output_directory_;

  /** INTERFACE **/
 public:
  BatchRunner(std::string name, jrl::Dataset dataset, std::string output_dir) : name_(name), dataset_(dataset) {
    // Setup the save out locations
    // Ensure that the output location exists
    bfs::create_directory(output_dir);
    // Generate the output dir
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::stringstream time_ss;
    time_ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    output_directory_ = output_dir + "/" + dataset.name() + "_" + name_ + "_" + time_ss.str();
    bfs::create_directory(output_directory_);

    // Generate Interation output folder
    iteration_output_directory_ = output_directory_ + "/iterations";
    bfs::create_directory(iteration_output_directory_);
  }

  /// @brief Run initialization code
  virtual BatchIterResults init() = 0;
  /// @brief Runs a single iteration of the relevant algorithm
  /// @return The estimate and consistent residual after running the specified algorithm
  virtual BatchIterResults iterate() = 0;
  /// @brief Returns true iff the method has fully converged
  virtual bool isConverged() = 0;
  /// @brief Returns the number of iterations of the runner required to communicate throughout the entire comm network
  virtual size_t iterationsPerFullComms() { return 1; }  // for all methods except MESA this is one

  /// @brief Run initialization code, potentially using non-default parameters from file
  void optimize();
};
}  // namespace batch_runners
#include "batch_runners/BatchRunner-inl.h"