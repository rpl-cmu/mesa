import glob
import os
import pickle

import gtsam
import jrl
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
from helpers.parse_results_directory import read_results_metrics_all

plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams["mathtext.fontset"] = "cm"
plt.rcParams["pdf.fonttype"] = 42


def get_comms_for_thresh(results_dir, thresh_percentile):
    ms_count_tuples = read_results_metrics_all(results_dir)
    all_mean_residual = np.array([tup[1].mean_residual for tup in ms_count_tuples])
    comm_counts = np.array([tup[0] for tup in ms_count_tuples])

    thresh = all_mean_residual[-1] + thresh_percentile * all_mean_residual[-1]
    thresh_index = np.argmin(np.abs(all_mean_residual - thresh))
    return comm_counts[thresh_index]

def aggregate_results(
    dataset_dir,
    result_dir,
    independent_variables,
    method,
    baseline,
):

    parser = jrl.Parser()

    # Setup storage for the results for each method
    aggregated_results = {}
    for iv in independent_variables:
        aggregated_results[iv] = {}
        aggregated_results[iv]["comm_counts"] = []
        aggregated_results[iv]["residuals"] = []
        aggregated_results[iv]["baseline_residuals"] = []
        aggregated_results[iv]["thresh_comms"] = []

    for iv in independent_variables:
        # Lets get all the datasets
        all_dataset_files = sorted(glob.glob(os.path.join(dataset_dir, iv, "*.jrl")))
        for dataset_file in all_dataset_files:
            # Parse the dataset
            dataset = parser.parseDataset(dataset_file, False)

            # Get the method result dir
            method_result_dir = os.path.join(result_dir, iv, method)
            print(os.path.join(method_result_dir, "{}*/".format(dataset.name())))
            method_dataset_result_dir = glob.glob(
                os.path.join(method_result_dir, "{}*/".format(dataset.name()))
            )[0]

            # Retrieve the baseline dir
            baseline_result_dir = os.path.join(result_dir, iv, baseline)
            baseline_dataset_result_dir = glob.glob(
                os.path.join(baseline_result_dir, "{}*/".format(dataset.name()))
            )[0]
            baseline_mean_residual = read_results_metrics_all(
                baseline_dataset_result_dir
            )[-1][1].mean_residual
            aggregated_results[iv]["baseline_residuals"].append(baseline_mean_residual)

            # Retrieve the method results
            comms_and_metrics = read_results_metrics_all(method_dataset_result_dir)
            thresh_comms = get_comms_for_thresh(method_dataset_result_dir, 0.01)
            comms, residual = [], []
            for cc, ms in comms_and_metrics:
                comms.append(cc)
                residual.append(ms.mean_residual)
            aggregated_results[iv]["comm_counts"].append(np.array(comms))
            aggregated_results[iv]["residuals"].append(np.array(residual))
            aggregated_results[iv]["thresh_comms"].append(thresh_comms)

    return aggregated_results


def stack_uneven(list_of_array):
    longest = max([a.shape[0] for a in list_of_array])
    padded_loa = []
    for arr in list_of_array:
        padded_loa.append(np.pad(arr, (0, longest - arr.shape[0]), "edge"))
    return np.stack(padded_loa)


def compare_average_convergence(
    dataset_dir,
    result_dir,
    independent_variables,
    method,
    baseline,
):

    pkl_file = os.path.join(result_dir, "{}_convergence_summary.pkl".format(method))
    if not os.path.exists(pkl_file):
        aggregated_results = aggregate_results(
            dataset_dir,
            result_dir,
            independent_variables,
            method,
            baseline,
        )
        with open(pkl_file, "wb") as handle:
            pickle.dump(aggregated_results, handle)
    else:
        with open(pkl_file, "rb") as pickle_file:
            aggregated_results = pickle.load(pickle_file)

    # Lets setup the figure
    fig, ax = plt.subplots(1, 1, figsize=[4, 2], dpi=200)
    ax.set_yscale("log")  # , linthresh=10)
    #ax.set_xscale("log") #, linthresh=1)

    colors = sns.color_palette("colorblind", len(independent_variables))
    for idx, iv in enumerate(independent_variables):
        longest_comms = np.array([])
        for cc in aggregated_results[iv]["comm_counts"]:
            if cc.shape[0] > longest_comms.shape[0]:
                longest_comms = cc

        residuals = stack_uneven(aggregated_results[iv]["residuals"])
        mean = np.mean(residuals, axis=0)
        stddev = np.std(residuals, axis=0)
        ax.plot(longest_comms, mean, label=iv, color=colors[idx])
        ax.axhline(
            np.mean(np.array(aggregated_results[iv]["baseline_residuals"])),
            color=colors[idx],
            linestyle="dashed",
            linewidth=0.5,
        )
        ax.fill_between(
            longest_comms, mean - stddev, mean + stddev, alpha=0.3, color=colors[idx]
        )

    # Residual
    ax.set_ylabel("Mean Residual")
    ax.set_xlabel("# Communications")
    ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))
    ax.set_xticklabels(independent_variables, minor=True)
    fig.tight_layout(pad=0.25)


    # Lets setup the second figure
    fig, ax = plt.subplots(1, 1, figsize=[4, 2], dpi=200)
    colors = sns.color_palette("colorblind", len(independent_variables))
    for idx, iv in enumerate(independent_variables):
        thresh_comms = np.array(aggregated_results[iv]["thresh_comms"])
        comms_std_dev = np.std(thresh_comms)
        print(iv, " : ", thresh_comms)
        ax.errorbar(iv, np.mean(thresh_comms), yerr=comms_std_dev, label=iv, marker='o', color=colors[idx], capsize=5)

    # Residual
    ax.set_ylabel("Comms to reach within 1% of final")
    ax.set_xlabel("Number of Robots")
    #ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))
    ax.set_xticklabels(independent_variables, minor=True)
    fig.tight_layout(pad=0.25)
    plt.grid(True, ls="-")

    plt.show()
