from utils.plotter import plot_variables, plot_grouped_subfigures
import matplotlib.pyplot as plt

# ---- Plot Enable Flags ----
PLOT_FLIGHT_PATH = False
PLOT_ROTOR_FORCES = False
PLOT_FORCES_MOMENTS = False
PLOT_YAW = False
PLOT_X_ACC = True
PLOT_Y_ACC = True
PLOT_X_SPEED = True
PLOT_Y_SPEED = True
PLOT_VERTICAL_SPEED = False
PLOT_ROLL_CONTROL = False
PLOT_PITCH_CONTROL = False
PLOT_EULER = True

# ---- Plotting ----
if PLOT_FLIGHT_PATH:
    plot_variables("log.csv", "Time", ["Altitude", "VerticalSpeed", "Acceleration", "XPos", "YPos", "EulerX", "EulerY", "EulerZ"], title_prefix="Helicopter", ylabel="Flight Variable")

if PLOT_ROTOR_FORCES:
    plot_variables("rotorLog.csv", "Time", ["RotorForceX", "RotorForceY", "RotorForceZ"], title_prefix="Rotor Forces", ylabel="Forces")
    plot_variables("rotorLog.csv", "Time", ["RotorMomentX", "RotorMomentY", "RotorMomentZ"], title_prefix="Rotor Moments", ylabel="Moments")

if PLOT_FORCES_MOMENTS:
    plot_variables("forcesAndMomentsLog.csv", "Time", ["ForceX", "ForceY", "ForceZ", "MomentX", "MomentY", "MomentZ"], title_prefix="Total Forces/Moments", ylabel="Forces & Moments")

if PLOT_YAW:
    plot_grouped_subfigures(
        "controllerLog.csv", "Time",
        [["TargetYaw", "Yaw"], ["TailRotor"]],
        [["Measured Yaw", "Target Yaw"], ["Pedal Input"]],
        xlabel="Time (s)", ylabel="Value", nrows=2, ncols=1
    )

if PLOT_X_ACC:
    plot_grouped_subfigures(
        "statesLog.csv", "Time",
        [["AccX"], ["Roll"]],
        [["X Acceleration"], ["Roll Input"]],
        xlabel="Time (s)", ylabel="Value", nrows=2, ncols=1
    )
if PLOT_Y_ACC:
    plot_grouped_subfigures(
        "statesLog.csv", "Time",
        [["AccY"], ["Pitch"]],
        [["Y Acceleration"], ["Pitch Input"]],
        xlabel="Time (s)", ylabel="Value", nrows=2, ncols=1
    )

if PLOT_X_SPEED:
    plot_grouped_subfigures(
        "statesLog.csv", "Time",
        [["VelX"], ["AccX"]],
        [["X Velocity"], ["AccX Input"]],
        xlabel="Time (s)", ylabel="Value", nrows=2, ncols=1
    )
if PLOT_Y_SPEED:
    plot_grouped_subfigures(
        "statesLog.csv", "Time",
        [["VelY"], ["AccY"]],
        [["Y Velocity"], ["AccY Input"]],
        xlabel="Time (s)", ylabel="Value", nrows=2, ncols=1
    )
if PLOT_VERTICAL_SPEED:
    plot_grouped_subfigures(
        "controllerLog.csv", "Time",
        [["Vz", "TargetVz"], ["Collective"]],
        [["Measured Vz", "Target Vz"], ["Collective Input"]],
        xlabel="Time (s)", ylabel="Value", nrows=2, ncols=1
    )

if PLOT_ROLL_CONTROL:
    plot_grouped_subfigures(
        "controllerLog.csv", "Time",
        [["Roll", "RollError"], ["LatCyclic"]],
        [["Measured Roll", "Roll Error"], ["Roll Input"]],
        xlabel="Time (s)", ylabel="Value", nrows=2, ncols=1
    )

if PLOT_PITCH_CONTROL:
    plot_grouped_subfigures(
        "controllerLog.csv", "Time",
        [["Pitch", "PitchError"], ["LongCyclic"]],
        [["Measured Pitch", "Pitch Error"], ["Pitch Input"]],
        xlabel="Time (s)", ylabel="Value", nrows=2, ncols=1
    )

if PLOT_EULER:
    plot_grouped_subfigures(
        "controllerLog.csv", "Time",
        [["Roll"], ["Pitch"], ["Yaw"]],
        [["Roll (deg)"], ["Pitch (deg)"], ["Yaw (deg)"]],
        xlabel="Time (s)", ylabel="Value", nrows=3, ncols=1
    )

plt.show(block=True)
input("Press Enter to close all plots and exit...")
plt.close("all")
