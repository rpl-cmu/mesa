import gtsam
import numpy as np
import matplotlib.pyplot as plt
import jrl
from helpers.parse_results_directory import read_results_metrics_all
from helpers.style_sheet import METHOD_STYLE_SHEET
import seaborn as sns

plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams["mathtext.fontset"] = "cm"
plt.rcParams["pdf.fonttype"] = 42


def compute_scaled_ate(metric_summary, trans_scale, rot_scale):
    scaled_ate = 0.0
    for rid in metric_summary.robots:
        scaled_ate += ((1.0 / trans_scale) * metric_summary.robot_ate[rid][0]) ** 2
        scaled_ate += (
            (1.0 / np.deg2rad(rot_scale)) * metric_summary.robot_ate[rid][1]
        ) ** 2
    return np.sqrt(scaled_ate / len(metric_summary.robots))


def compare_convergence(
    dataset_file,
    constant_method_result_dirs,
    method_result_dirs,
):
    parser = jrl.Parser()
    # Load the dataset
    dataset = parser.parseDataset(dataset_file, False)

    # Lets setup the figure
    fig, axes = plt.subplots(1, 1)
    # fig.suptitle(dataset.name(), fontsize=16)

    for const_result_dir in constant_method_result_dirs:
        ms = read_results_metrics_all(const_result_dir)[-1][1]
        axes.axhline(
            ms.mean_residual,  # residual
            label=METHOD_STYLE_SHEET[ms.method_name]["name"],
            color=METHOD_STYLE_SHEET[ms.method_name]["color"],
        )

    # Lets itertotal_ate[0] over all the different BASELINES
    for mthd_result_dir in method_result_dirs:
        comms_and_metrics = read_results_metrics_all(mthd_result_dir)
        name = comms_and_metrics[0][1].method_name
        comms = []
        residual = []
        for cc, ms in comms_and_metrics:
            if ms.total_ate[0] < np.inf and ms.total_ate[0] != np.NaN:
                # print(ms.mean_residual)
                comms.append(cc)
                residual.append(ms.mean_residual)
        axes.plot(
            comms,
            residual,
            label=METHOD_STYLE_SHEET[name]["name"],
            color=METHOD_STYLE_SHEET[name]["color"]
        )

    axes.set_ylabel("Mean Residual")
    axes.set_xlabel("# Communications")
    axes.set_yscale("log")
    axes.legend(loc="center left", bbox_to_anchor=(1, 0.5))
    fig.tight_layout(pad=0.25)
    plt.show()
