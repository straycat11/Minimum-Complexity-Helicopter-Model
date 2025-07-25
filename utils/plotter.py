import pandas as pd
import math
import matplotlib.pyplot as plt

def plot_variables(csv_file, x_var, y_vars, title_prefix="Plot of", xlabel=None, ylabel="Value", legend_labels=None):
    df = pd.read_csv(csv_file)

    plt.figure()
    for i, y_var in enumerate(y_vars):
        label = legend_labels[i] if legend_labels and i < len(legend_labels) else y_var
        plt.plot(df[x_var], df[y_var], label=label)

    plt.title(f"{title_prefix} {' vs '.join(legend_labels or y_vars)}")
    plt.xlabel(xlabel or x_var)
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.show(block=False)
    plt.pause(0.1)

def plot_grouped_subfigures(
    csv_file,
    x_var,
    y_groups,
    group_titles=None,
    group_labels=None,
    xlabel=None,
    ylabel="Value",
    nrows=None,
    ncols=None
):
    df = pd.read_csv(csv_file)
    num_groups = len(y_groups)

    if not nrows or not ncols:
        ncols = math.ceil(math.sqrt(num_groups)) if not ncols else ncols
        nrows = math.ceil(num_groups / ncols) if not nrows else nrows

    fig, axs = plt.subplots(nrows, ncols, figsize=(5 * ncols, 3.5 * nrows))
    axs = axs.flatten() if num_groups > 1 else [axs]

    for i, group in enumerate(y_groups):
        ax = axs[i]
        labels = group_labels[i] if group_labels and i < len(group_labels) else group
        title = group_titles[i] if group_titles and i < len(group_titles) else f"Group {i+1}"

        for j, y_var in enumerate(group):
            label = labels[j] if j < len(labels) else y_var
            ax.plot(df[x_var], df[y_var], label=label)

        ax.set_title(title)
        ax.set_xlabel(xlabel or x_var)
        ax.set_ylabel(ylabel)
        ax.grid(True)
        ax.legend()

    for j in range(len(y_groups), len(axs)):
        axs[j].axis("off")

    plt.tight_layout()
    plt.show(block=False)