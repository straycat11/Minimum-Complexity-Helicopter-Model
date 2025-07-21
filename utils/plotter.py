import pandas as pd
import matplotlib.pyplot as plt

def plot_altitude(csv_file):
    df = pd.read_csv(csv_file)
    plt.plot(df["Time"], df["Altitude"])
    plt.title("Helicopter Altitude Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.grid(True)
    plt.show()
