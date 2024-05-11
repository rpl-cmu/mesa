# Scripts

This directory contains helper scripts for running experiments and visualizing results. Namely we provide:
* `run-experiment` - A script for running a large experiment (i.e. multiple opt methods on many datasets).
* `plot-results` - A script for plotting the optimized trajectory in a JRR results file.
* `plot-jrl` - Plots a JRL dataset for visualization.
* `make-multi-robot-dataset-linear` - Generates a random linear (no rotations) dataset.
* `make-multi-robot-dataset-3d` - Generates a random synthetic 3d nonlinear dataset.
* `make-multi-robot-dataset-3d` - Generates a random synthetic 2d nonlinear dataset.
* `compare-convergence` - Plots the convergence curves of multiple optimization methods.
* `compare-avg-final-results` - Summarizes the results from `run-experiment` in terms of the final achieved results. 
* `compare-average-convergence` - Summarizes the results from `run-experiment` when they converge to within 1% of the final value (requires we run the experiment with very tight convergence thresholds)
* `compare-acc-to-comms` - Summarizes the results from `run-experiment` plotting average # communications against the achieved mean residual.

For each script see their `--help` documentation and in-line comments for usage and additional information.

WARNING: These scripts are provided as-is and you may run-into issues. As they are only used for visualization they are messy and frequently modified for different purposes. Their current functionality is untested, but should generate plots pretty similar to those found in our paper.