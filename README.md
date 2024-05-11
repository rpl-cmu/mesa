# MESA
 [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)  &nbsp;[<img src="media/rpl.png" height=20px>](https://rpl.ri.cmu.edu/)

MESA is a fully distributed, asynchronous, and general purpose optimization algorithm for Consensus Simultaneous Localization and Mapping (CSLAM). Multi-robot teams require that agents have timely and accurate solutions to their state as well as the states of the other robots in the team. To optimize this solution we develop a CSLAM back-end based on Consensus ADMM called MESA (Manifold, Edge-based, Separable ADMM). MESA is fully distributed to tolerate failures of individual robots, asynchronous to tolerate communication delays and outages, and general purpose to handle any CSLAM problem formulation. We demonstrate that MESA exhibits superior convergence rates and accuracy compare to existing state-of-the art CSLAM back-end optimizers.

<p align="center">
<img src="media/sphere_solve.gif"
    alt="MESA solving a partioned version of the benchmark sphere dataset." 
    width="80%"/>
</p>

This code implements the MESA algorithm as described in our ICRA 2024 paper which can be found on [arXiv](https://arxiv.org/abs/2310.12320). If you use this package please cite our paper as:

```
@inproceedings{mcgann_mesa_2024, 
    title = {Asynchronous Distributed Smoothing and Mapping via On-Manifold Consensus {ADMM}},
    author = {D. McGann and K. Lassak and M. Kaess},
    fullauthor = {Daniel McGann and Kyle Lassak and Michael Kaess},
    booktitle = {Proc. IEEE Intl. Conf. on Robotics and Automation (ICRA)},
    year = 2024,
    pages = {n/a}, % will be added soon
    address = {Yokohama, {JP}}
}
```

## Project Structure
The package is broken down into 4 main components:
 - `mesa/` - Contains the source code for the MESA algorithm
 - `experiments/` - Contains the code to run the experiments in our paper.
 - `thirdparty/` - Contains implementations of prior works included via submodules.
 - `scripts/` - Helper scripts to analyze experiments and visualize results. [requires python + bindings]

 Each component contains its own readme with additional information.

## Install Instructions
These install instructions were tested on Ubuntu 20.04 as of April, 2024.

### 1. Setup the workspace
* In a good location on your machine construct a workspace (e.g. `mesa_workspace`) to house the various dependencies for this package.
    * `cd /some/path/` and `mkdir mesa_workspace`
* We will assume that `mesa_workspace` is the name of your working directory for the rest of these instructions
* [Optional for scripts] Next construct a python environment for this workspace. We recommend using conda.
    * Activate the environment
    * Install `numpy matplotlib scipy setuptools`
    * Note: This is needed only to run the python scripts in `scripts/`

### 2. Dependency 1 - GTSAM
* In the workspace directory clone gtsam from fork:
    * `git clone -b mesa_workspace git@github.com:DanMcGann/gtsam.git`
    * NOTE: You must be on branch `mesa_workspace`
* Enter `gtsam/`
    * `cd gtsam`
* Build GTSAM
    * `mkdir build`
    * `cd build`
    * `cmake .. -DGTSAM_BUILD_PYTHON=1`
    * `make`
* [Optional for scripts] Install GTSAM python into the project's python environment
    * `make python-install`

### 3. Dependency 2 - JRL
* Install JRL dependency `nlohmann-json`
    * `sudo apt-get install nlohmann-json3-dev`
* In the workspace directory clone JRL
    * `git clone --recurse-submodules  git@github.com:DanMcGann/jrl.git`
* Enter JRL and build
    * `cd jrl`
    * `mkdir build`
    * `cd build`
    * `cmake .. -DGTSAM_DIR=/path/to/mesa_workspace/gtsam/build -DGTSAM_INCLUDE_DIR=/path/to/mesa_workspace/gtsam/gtsam`
    * `make`
* [Optional for scripts] Install JRL python into the project's python environment
    * `make python-install`

### 4. MESA
* In the workspace directory clone MESA
    * `git clone --recurse-submodules  git@github.com:rpl-cmu/mesa.git`
* Enter MESA and build
    * `cd mesa`
    * `mkdir build && cd build`
    * `cmake .. -Djrl_DIR=/path/to/jrl/build -Djrl_INCLUDE_DIR=/path/to/jrl/include -DGTSAM_DIR=/path/to/gtsam/build -DGTSAM_INCLUDE_DIR=/path/to/gtsam/gtsam`


## Dataset Info
In this project we use JRL (Json Robot Log) format datasets due to their improvements over alternatives like g2o or toro. For more info on JRL see [here](https://github.com/DanMcGann/jrl). Inline with the JRL format definition we assume that variable keys use characters to identify robots and that all variables "owned" by a robot are indexed as such. Since robots share variables in the multi-robot case a robot's local factor graph will include variables "owned" by other robots and those variables will be indexed using the owner's character. 

Note: Since MESA and the prior works implemented in this project are batch methods we do not use the incremental nature of the JRL format. Rather our runner interface aggregates all measurements in a JRL file so the methods can optimize the problem in batch.

## Run Experiment instructions
Below are example instructions to use this package to generate a dataset and run different optimization methods.


### 1. Generate a Dataset
First we need to generate an example dataset. From the `mesa_workspace` directory you can run the following command. Note this command is the same (apart from paths + number repetitions + name) as the command used to generate the 200 length datasets in the length experiment from our ICRA paper.

```
./mesa/scripts/make-multi-robot-3d-dataset -o . -nr 4 -np 200 --loop_closure_index_threshold 10 --loop_closure_distance_threshold 5 --loop_closure_probability 0.4 --comm_range 30 --comm_freq 10 --odom_noise_sigmas 0.175 0.175 0.175 0.05 0.05 0.05 --loop_noise_sigmas 0.175 0.175 0.175 0.05 0.05 0.05 --comm_loop_noise_sigmas 0.175 0.175 0.175 0.05 0.05 0.05 --prior_noise_sigmas 0.0175 0.0175 0.0175 0.01 0.01 0.01 --robot_zero_prior_noise_sigmas 0.0175 0.0175 0.0175 0.01 0.01 0.01 --xlims -30 30 --ylims -30 30 --zlims -10 10 -n example_dataset -r 1
```

### 2. Use the main runner to run a method
* `./mesa/build/experiments/run-dist-batch -i example_dataset_0000.jrl -m centralized -o . --is3d`
* `./mesa/build/experiments/run-dist-batch -i example_dataset_0000.jrl -m geodesic-mesa -o . --is3d`

### 3. Plot the results
* `./scripts/plot-results -d example_dataset_0000.jrl -r example_dataset_0000_geodesic-mesa_<DATE>/final_results.jrr.cbor --is3d `

### 4. Visualize Convergence Curve
* `./scripts/compare-convergence -d example_dataset_0000.jrl  -c example_dataset_0000_centralized_<DATE>/ -m example_dataset_0000_geodesic-mesa_<DATE>/ -rs 1 -ts 1`


## Issues
If you have any issues with the code please submit a bug report here on github!