# plotter.py
import pandas as pd
import matplotlib.pyplot as plt

def plot_variables(csv_file, x_var, y_vars, title_prefix="Plot of", xlabel=None, ylabel="Value"):
    df = pd.read_csv(csv_file)

    for y_var in y_vars:
        plt.figure()
        plt.plot(df[x_var], df[y_var], label=y_var)
        plt.title(f"{title_prefix} {y_var}")
        plt.xlabel(xlabel or x_var)
        plt.ylabel(ylabel)
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        plt.show(block=False)   # ✅ Non-blocking
        plt.pause(0.1)          # ✅ Give time for rendering

    input("Press Enter to close all plots...")  # So the script doesn't quit instantly
    plt.close('all')
