import glob
import os
import pickle

import gtsam
import jrl
import matplotlib.pyplot as plt
import matplotlib.ticker as tck
import numpy as np
from helpers.style_sheet import METHOD_STYLE_SHEET
from helpers.plot_summaries import boxplot, finish_boxplot_axes, plot_boxplot_symbol

plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams["mathtext.fontset"] = "cm"
plt.rcParams['pdf.fonttype'] = 42

import codecs

def aggregate_results(
    dataset_dir,
    result_dir,
    independent_variables,
    all_methods,
):

    parser = jrl.Parser()

    # Setup storage for the results for each method
    aggregated_results = {}
    for iv in independent_variables:
        aggregated_results[iv] = {}
        for method in all_methods:
            aggregated_results[iv][method] = {
                "ate_trans": [],
                "ate_rot": [],
                "mean_residual": [],
            }
    for iv in independent_variables:
        # Lets get all the datasets
        all_dataset_files = sorted(glob.glob(os.path.join(dataset_dir, iv, "*.jrl")))
        for dataset_file in all_dataset_files:
            # Parse the dataset
            dataset = parser.parseDataset(dataset_file, False)

            for method in all_methods:
                method_result_dir = os.path.join(result_dir, iv, method)
                print(os.path.join(method_result_dir, "{}*/".format(dataset.name())))
                method_dataset_result_dir = glob.glob(
                    os.path.join(method_result_dir, "{}*/".format(dataset.name()))
                )[0]
                method_dataset_file_result_file = os.path.join(
                    method_dataset_result_dir, "final_metrics.jrm.cbor"
                )
                if os.path.isfile(method_dataset_file_result_file):
                    ms = parser.parseMetricSummary(
                        method_dataset_file_result_file, True
                    )

                    aggregated_results[iv][method]["ate_trans"].append(ms.total_ate[0])
                    aggregated_results[iv][method]["ate_rot"].append(ms.total_ate[1])
                    aggregated_results[iv][method]["mean_residual"].append(
                        ms.mean_residual
                    )

                else:
                    print(
                        "method: {} failed for dataset: {}".format(
                            method, dataset.name()
                        )
                    )
    return aggregated_results


def compare_average_final_results(
    experiment_name,
    dataset_dir,
    result_dir,
    independent_variables,
    independent_variable_labels,
    all_methods,
    xlabel,
    legend,
    output
):

    pkl_file = os.path.join(result_dir, "metric_summary.pkl")
    if not os.path.exists(pkl_file):
        aggregated_results = aggregate_results(
            dataset_dir,
            result_dir,
            independent_variables,
            all_methods,
        )
        with open(pkl_file, "wb") as handle:
            pickle.dump(aggregated_results, handle)
    else:
        with open(pkl_file, "rb") as pickle_file:
            aggregated_results = pickle.load(pickle_file)

    # Lets setup the figure
    fig, ax = plt.subplots(1, 1, figsize=[4 if not legend else 6, 2 if experiment_name == "" else 2.5], dpi=200)
    ax.set_yscale("log")  # , linthresh=10)
    fig.suptitle(experiment_name, fontsize=12)

    for j, iv in enumerate(independent_variables):
        for i, method in enumerate(all_methods):
            boxplot(
                aggregated_results[iv][method]["mean_residual"],
                ax,
                j,
                i,
                len(all_methods),
                METHOD_STYLE_SHEET[method]["color"],
                linestyle=METHOD_STYLE_SHEET[method]["linestyle"],
                width=0.15
            )

    finish_boxplot_axes(ax, independent_variables)

    for j, iv in enumerate(independent_variables):
        for i, method in enumerate(all_methods):
            plot_boxplot_symbol(
                ax,
                j,
                i,
                len(all_methods),
                METHOD_STYLE_SHEET[method]["color"],
                symbol=METHOD_STYLE_SHEET[method]["symbol"],
                symbol_size=6,
                label=METHOD_STYLE_SHEET[method]["name"] if j == 0 else None,
                linestyle="" #METHOD_STYLE_SHEET[method]["linestyle"],
            )

    # Residual
    ax.set_ylabel("Mean Residual")
    ax.set_xlabel(xlabel)
    
    # The following 3 lines were used for the measurement type experiment plot
    #ax.yaxis.set_major_locator(tck.LogLocator(base=100.0, numticks=5))
    #x.yaxis.set_minor_locator(tck.LogLocator(base=100.0, numticks=1000 ,subs=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9)))
    #plt.setp(ax.get_yminorticklabels(), visible=False)
    
    if (legend):
        ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))
    ax.set_xticklabels(independent_variable_labels, minor=True)
    print(independent_variable_labels[0])
    fig.tight_layout(pad=0.25)

    if output:
        plt.savefig(output)

    plt.show()
