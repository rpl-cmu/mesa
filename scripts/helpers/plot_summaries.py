import matplotlib.pyplot as plt
import numpy as np


def finish_boxplot_axes(ax: plt.Axes, major_vars):
    # Set the x axis
    ax.set_xticks(np.array(range(len(major_vars) + 1)) - 0.5)
    ax.set_xticklabels("")
    ax.set_xticks(range(len(major_vars)), minor=True)
    ax.set_xlim([-0.6, len(major_vars) - 1 + 0.6])
    ax.grid(b=True, which="major")
    ax.grid(b=True, which="both", axis="y")
    ax.tick_params(axis="y", which="both", labelsize=9)


def plot_boxplot_symbol(
    ax,
    major_idx: int,
    minor_index: int,
    minor_num: int,
    color,
    symbol,
    symbol_size,
    label,
    linestyle,
):
    ylow, yhigh = ax.get_ylim()
    ax.set_ylim([ylow, yhigh])
    
    ax.plot(
        [major_idx - 0.5 + ((minor_index + 0.5) / minor_num)],
        1.05,
        linestyle=linestyle,
        marker=symbol,
        color=color,
        markersize=symbol_size,
        clip_on=False,
        zorder=100,
        label=label,
        transform=ax.get_xaxis_transform()
    )


def boxplot(
    data: np.ndarray,
    ax: plt.Axes,
    major_idx: int,
    minor_index: int,
    minor_num: int,
    color,
    line_width=1.5,
    linestyle="solid",
    width=0.08,
    flier_markersize=3,
    flier_marker="+",
):
    """
    :param: data - array of scalars for which we are computing the boxplot
    :param: ax - The axes on which the boxplot will be plotted
    :param: major_idx = the major index (Independent Variable Index)
    :param: minor_index = the minor index (Method index)
    :param: major_num = the magnitude of the minor index (ex number of independent vars)
    :param: minor_num = the magnitude of the major index (ex number of methods)
    :param: color - the color associated with the minor index
    :param: line_width - line width for the boxplot lines
    :param: width - The width of the boxplot
    :param: flier_markersize - Size of marker for boxplot outliers (fliers)
    :param: flier_marker - The marker for boxplot outliers (fliers)
    :param: symbol - the symbol associated with the minor index (plotted above the axes)
    :param: symbol_pos - the position above the axes for the symbol (plotted above the axes)
    :param: symbol_size - the size above the minor index symbol (plotted above the axes)
    """
    line_props = {
        "color": color,
        "linestyle": linestyle,
        "lw": line_width,
        "ms": 0,
        "marker": "o",
    }
    flier_props = {
        "markerfacecolor": color,
        "markeredgecolor": color,
        "markersize": flier_markersize,
        "marker": flier_marker,
    }
    ax.boxplot(
        data,
        notch=False,
        widths=[width],
        boxprops=line_props,
        whiskerprops=line_props,
        medianprops=line_props,
        meanprops=line_props,
        capprops=line_props,
        flierprops=flier_props,
        positions=np.array([major_idx - 0.5 + ((minor_index + 0.5) / minor_num)]),
        showfliers=True,
    )
