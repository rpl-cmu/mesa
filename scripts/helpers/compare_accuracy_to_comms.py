import glob
import os
import pickle

import jrl
import matplotlib.pyplot as plt
from matplotlib.ticker import Locator

import numpy as np
from helpers.style_sheet import METHOD_STYLE_SHEET
from helpers.parse_results_directory import read_results_metrics_all
from matplotlib.patches import Ellipse

plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams["mathtext.fontset"] = "cm"
plt.rcParams["pdf.fonttype"] = 42


class MinorSymLogLocator(Locator):
    """
    Dynamically find minor tick positions based on the positions of
    major ticks for a symlog scaling.
    """

    def __init__(self, linthresh):
        """
        Ticks will be placed between the major ticks.
        The placement is linear for x between -linthresh and linthresh,
        otherwise its logarithmically
        """
        self.linthresh = linthresh

    def __call__(self):
        "Return the locations of the ticks"
        majorlocs = self.axis.get_majorticklocs()

        # iterate through minor locs
        minorlocs = []

        # handle the lowest part
        for i in range(1, len(majorlocs)):
            majorstep = majorlocs[i] - majorlocs[i - 1]
            if abs(majorlocs[i - 1] + majorstep / 2) < self.linthresh:
                ndivs = 10
            else:
                ndivs = 9
            minorstep = majorstep / ndivs
            locs = np.arange(majorlocs[i - 1], majorlocs[i], minorstep)[1:]
            minorlocs.extend(locs)

        return self.raise_if_exceeds(np.array(minorlocs))

    def tick_values(self, vmin, vmax):
        raise NotImplementedError(
            "Cannot get tick locations for a " "%s type." % type(self)
        )


def find_comms_thresh(method, results_dir, thresh_percentile):
    ms_count_tuples = read_results_metrics_all(results_dir)
    all_mean_residual = np.array([tup[1].mean_residual for tup in ms_count_tuples])
    comm_counts = np.array([tup[0] for tup in ms_count_tuples])

    thresh = all_mean_residual[-1] + thresh_percentile * all_mean_residual[-1]
    thresh_index = np.argmin(np.abs(all_mean_residual - thresh))

    return np.array([all_mean_residual[thresh_index], comm_counts[thresh_index]])
    """    
        plt.figure()
        ax = plt.gca()
        ax.set_xlim([0, 6000])
        ax.set_xlabel(method)
        plt.plot(comm_counts, all_sate)
        ax.axhline(all_sate[thresh_index], color='black')
        ax.axvline(comm_counts[thresh_index], color='gray')
        plt.show()
    """


def aggregate_results(
    dataset_dir,
    result_dir,
    independent_variables,
    all_methods,
    constant_methods,
    thresh,
):

    parser = jrl.Parser()

    # Setup storage for the results for each method
    aggregated_results = {}
    for iv in independent_variables:
        aggregated_results[iv] = {}
        for method in all_methods:
            aggregated_results[iv][method] = []
        for const_mthd in constant_methods:
            aggregated_results[iv][const_mthd] = []

    for iv in independent_variables:
        # Lets get all the datasets
        all_dataset_files = sorted(glob.glob(os.path.join(dataset_dir, iv, "*.jrl")))
        for dataset_file in all_dataset_files:
            # Parse the dataset
            dataset = parser.parseDataset(dataset_file, False)
            for method in all_methods:
                method_result_dir = os.path.join(result_dir, iv, method)
                method_dataset_result_dir = glob.glob(
                    os.path.join(method_result_dir, "{}*/".format(dataset.name()))
                )
                if len(method_dataset_result_dir) > 0:
                    aggregated_results[iv][method].append(
                        find_comms_thresh(method, method_dataset_result_dir[0], thresh)
                    )

            for const_mthd in constant_methods:
                const_mthd_result_dir = os.path.join(result_dir, iv, const_mthd)
                const_mthd_dataset_result_dir = glob.glob(
                    os.path.join(const_mthd_result_dir, "{}*/".format(dataset.name()))
                )[0]
                ms = read_results_metrics_all(const_mthd_dataset_result_dir)[-1][1]
                aggregated_results[iv][const_mthd].append(ms.mean_residual)

    return aggregated_results


def confidence_ellipse(x, y, ax, n_std, color, symbol, symbol_size, label, linestyle):
    if x.size != y.size:
        raise ValueError("x and y must be the same size")

    cov = np.cov(x, y)
    mean_x, mean_y = np.mean(x), np.mean(y)
    lambda_, v = np.linalg.eig(cov)
    lambda_ = np.sqrt(lambda_)
    ellipse = Ellipse(
        xy=(mean_x, mean_y),
        width=lambda_[0] * n_std * 2,
        height=lambda_[1] * n_std * 2,
        angle=np.rad2deg(np.arccos(v[0, 0])),
        facecolor=(0, 0, 0, 0),
        edgecolor=color,
        linestyle=linestyle,
        zorder=2
    )
    ax.add_patch(ellipse)
    # ax.scatter(x, y, color=color, marker=".")
    ax.plot(
        [mean_x],
        [mean_y],
        color=color,
        marker=symbol,
        label=label,
        linestyle=linestyle,
        markersize=symbol_size,
    )


def compare_accuracy_to_comms(
    experiment_name,
    dataset_dir,
    result_dir,
    independent_variables,
    all_methods,
    constant_methods,
    thresh,
    legend,
    output,
):

    pkl_file = os.path.join(
        result_dir, "acc_comm_metric_summary_{:03f}.pkl".format(thresh)
    )
    if not os.path.exists(pkl_file):
        aggregated_results = aggregate_results(
            dataset_dir,
            result_dir,
            independent_variables,
            all_methods,
            constant_methods,
            thresh,
        )
        with open(pkl_file, "wb") as handle:
            pickle.dump(aggregated_results, handle)
    else:
        with open(pkl_file, "rb") as pickle_file:
            aggregated_results = pickle.load(pickle_file)

    for j, iv in enumerate(independent_variables):
        # Lets setup the figure
        fig, ax = plt.subplots(
            1,
            1,
            figsize=[4 if not legend else 6, 2.25 if experiment_name == "" else 3],
            dpi=200,
        )
        fig.suptitle(experiment_name, fontsize=16)

        for const_mthd in constant_methods:
            mean_residuals = np.array(aggregated_results[iv][const_mthd])
            avg_const_mean_residual = np.mean(mean_residuals)
            stddev_const_mean_residual = np.std(mean_residuals)
            # ax.axhspan(
            #    avg_const_mean_residual - stddev_const_mean_residual,
            #    avg_const_mean_residual + stddev_const_mean_residual,
            #    facecolor=METHOD_STYLE_SHEET[const_mthd]["color"],
            #    alpha=0.5,
            # )
            ax.axhline(
                avg_const_mean_residual,  # residual
                color=METHOD_STYLE_SHEET[const_mthd]["color"],
                zorder=1,
            )
            ax.plot(
                [1.02],
                avg_const_mean_residual,
                linestyle=METHOD_STYLE_SHEET[const_mthd]["linestyle"],
                marker=METHOD_STYLE_SHEET[const_mthd]["symbol"],
                color=METHOD_STYLE_SHEET[const_mthd]["color"],
                markersize=6,
                clip_on=False,
                zorder=100,
                label=METHOD_STYLE_SHEET[const_mthd]["name"],
                transform=ax.get_yaxis_transform(),
            )

        for i, method in enumerate(all_methods):
            print(aggregated_results[iv][method])
            method_results = np.stack(aggregated_results[iv][method])
            print(method_results.T[0], method_results.T[1])
            # plt.scatter(method_results.T[1], method_results.T[0], label=method)
            confidence_ellipse(
                method_results.T[1],
                method_results.T[0],
                ax,
                n_std=3,
                symbol_size=4,
                color=METHOD_STYLE_SHEET[method]["color"],
                symbol=METHOD_STYLE_SHEET[method]["symbol"],
                label=METHOD_STYLE_SHEET[method]["name"],
                linestyle=METHOD_STYLE_SHEET[method]["linestyle"],
            )

        ax.tick_params(which="both", labelsize=9)
        # X-Axis Config
        ax.set_xlabel(
            "Communications".format(thresh * 100) # + r" $r_{mean}^2$"
        )
        # ax.set_xscale("symlog", linthresh=100)

        # Y-Axis Config
        ax.set_ylabel("Mean Residual".format(thresh * 100))
        # ax.set_ylim([1500, None])
        ax.set_yscale("log")
        #ax.set_yscale("symlog", linthresh=3500)
        ax.yaxis.set_minor_locator(MinorSymLogLocator(1e1))
        ax.grid(b=True, which="major", axis="both")
        ax.grid(b=True, which="both", axis="y")
        ax.set_axisbelow(True)

        if legend:
            ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))
        plt.tight_layout(pad=0.25)

        if output:
            plt.savefig(output)
        plt.show()
