import glob
import multiprocessing as mp
import os
import subprocess

"""
Runs an experiment. Use this module on a directory with the following structure
/datasets
    /"Indep_Var_1"
        indep_var_dataset_0.jrr
        ...
    ...
/results

And the script will produce results for all provided methods on all datasets for each independent variable
"""

HELPERS_DIR = os.path.dirname(os.path.abspath(__file__))
BUILD_DIR = os.path.join(HELPERS_DIR, "..", "..", "build")
RUNNER = os.path.join(BUILD_DIR, "experiments", "run-dist-batch")


def worker(method, dataset_file, output_dir, is3d, linear):
    command = "{} -m {} -i {} -o {} {} {}".format(
        RUNNER,
        method,
        dataset_file,
        output_dir,
        "--is3d" if is3d else "",
        "--linear" if linear else "",
    )
    subprocess.call(command.split(" "))


def run_experiment(experiment_dir, independent_vars, methods, nworkers, linear, is3d):
    # Datasets and Results Directory
    datasets_dir = os.path.join(experiment_dir, "datasets")
    results_dir = os.path.join(experiment_dir, "results")
    os.makedirs(results_dir, exist_ok=True)

    # Setup the multiprocessing pool
    pool = mp.Pool(nworkers)

    # Iterate over the independent variable
    for indep_var in independent_vars:
        # Setup variable directories
        indep_var_dataset_dir = os.path.join(datasets_dir, indep_var)
        indep_var_results_dir = os.path.join(results_dir, indep_var)
        os.makedirs(indep_var_results_dir, exist_ok=True)

        # Get all variable datasets
        indep_var_dataset_files = glob.glob(
            os.path.join(indep_var_dataset_dir, "*.jrl")
        )

        # For all methods and for all files
        for mthd in methods:
            indep_var_method_results_dir = os.path.join(indep_var_results_dir, mthd)
            for idv_dataset_file in indep_var_dataset_files:
                pool.apply_async(
                    worker,
                    args=(
                        mthd,
                        idv_dataset_file,
                        indep_var_method_results_dir,
                        is3d,
                        linear,
                    ),
                )

    pool.close()
    pool.join()