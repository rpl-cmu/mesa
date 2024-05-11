# Experiments

This module of MESA implements the executables used to run experiments. Specifically, it provides the main entry point `run-dist-batch`. Additionally, this module provides "runner" classes (in `batch_runners/`) for MESA and the prior works that we explored. The purpose of these "runners" is to implement all methods under a unified interface to simplify the running of experiments. Finally, this module provides a utility program `g2o-2-mr-jrl` that partitions an existing g2o dataset using METIS partitioning into a JRL format dataset for use with `run-dist-batch`.


## Run Distributed Optimization Batch
The `run-dist-batch` script is the main entry point to using this package. It will run any of the implemented optimization routines on a given JRL dataset. 

For a list of all supported optimization routines see `batch_runners/include/Factory.h`. 

This script will output (into the current directory) a results folder containing:
* `final_results.jrr.cbor` - The final optimization results in compressed JRR format.
* `final_metrics.jrr.cbor` - The final optimization metrics (ATE, Mean Residual etc.) in compressed JRM format.
* `communication_counts.txt` - A space separated list of values. Each is the cumulative number of communications used by the algorithms. Each entry corresponds to a saved iteration in `iterations/` except the last which corresponds to the final results.
* `iterations/` - Directory containing results after a subset of the optimization iterations. For each saved iteration there is a results and metrics file.