import time
import numpy as np
from models.helicopter import Helicopter
from models.atmosphere import Atmosphere
from models.integrator import euler6dof_step
from utils.logger import Logger
from utils.plotter import plot_variables
from utils.plotter import plot_grouped_subfigures
from config.params import params
from controllers.helicopter_controller import HelicopterController
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def freeze_channels(state):
    # Freeze longitudinal (X), lateral (Y), vertical (Z) translation
    # state["position"][0] = 0.0  # X position
    state["position"][1] = 0.0  # Y position
    # state["position"][2] = 0.0  # Z position (altitude)
    # state["body_velocity"][0] = 0.0  # longitudinal speed
    state["body_velocity"][1] = 0.0  # lateral speed
    # state["body_velocity"][2] = 0.0  # vertical speed
    # state["earth_velocity"][0] = 0.0
    state["earth_velocity"][1] = 0.0
    # state["earth_velocity"][2] = 0.0

    # Freeze roll, pitch and yaw angles (phi, theta, psi) and rates
    state["attitude"][0] = 0.0  # roll angle (phi)
    # state["attitude"][1] = 0.0  # pitch angle (theta)
    # state["attitude"][2] = 0.0  # yaw angle (psi)
    state["angular_rate"][0] = 0.0  # roll rate (p)
    # state["angular_rate"][1] = 0.0  # pitch rate (q)
    # state["angular_rate"][2] = 0.0  # yaw rate (r)
    state["angular_acceleration"][0] = 0.0
    # state["angular_acceleration"][1] = 0.0
    # state["angular_acceleration"][2] = 0.0

atmosphere = Atmosphere() 
logger = Logger(variable_names=["Time", "Altitude", "VerticalSpeed", "Acceleration", "XPos", "YPos", "ZPos", "EulerX", "EulerY", "EulerZ"])
loggerRotor = Logger(variable_names=["Time", "RotorForceX", "RotorForceY", "RotorForceZ", "RotorMomentX", "RotorMomentY", "RotorMomentZ"])
loggerForcesAndMoments = Logger(variable_names=["Time", "ForceX", "ForceY", "ForceZ", "MomentX", "MomentY", "MomentZ"])
controller = HelicopterController()
loggerControl = Logger(variable_names=[
    "Time",
    "TargetYaw", "Yaw", "YawError", "TailRotor",
    "TargetVz", "Vz", "VzError", "Collective",
    "Roll", "RollError", "LatCyclic",
    "Pitch", "PitchError", "LongCyclic"
])
loggerStates = Logger(variable_names=["Time", "AccX", "AccY", "AccZ",
                                "VelX", "VelY", "VelZ",
                                "Roll", "Pitch"])

target_state = {
    "yaw": 0.0,           # rad
    "vz": 0.0,            # m/s
    "vx": 0.0,            # m/s 
    "vy": 0.0          # m/s
}

# Define initial state and parameters (these are just placeholders)
initial_euler = np.deg2rad([-3.9, 5.12, 0.0])
previous_state = {
    "position": [0.0, 0.0, 0.0],
    "body_velocity": [0.0, 0.0, 0.0],
    "earth_velocity": [0.0, 0.0, 0.0],
    "body_acceleration": [0.0, 0.0, 0.0],
    "angular_acceleration": [0.0, 0.0, 0.0],
    "attitude": initial_euler,
    "attitude_q": (R.from_euler('zyx',initial_euler)).as_quat(),
    "angular_rate": [0.0, 0.0, 0.0],
    "vi_mr_prev": 33.44,
    "gv_7_prev": 0.0,
    "gv_8_prev": 0.0,
    "gr_7_prev": 0.0,
    "gr_8_prev": 0.0,
    "vi_tr_prev": 30.0
}
heli = Helicopter(params)

# Define parameters (replace with actual values)
F = np.array([0.0, 0.0, 10.0])  # lb
M = np.array([0.0, 1000.0, 0.0])  # lb
m = 5401.0 / 32.2  # slug
I = np.array([[2593.0, 0.0, 0.0], [0.0, 14320.0, 0.0], [0.0, 0.0, 12330.0]])  # Moment of inertia matrix
dt = 0.01  # Time step
a1, a2 = 1.5, 0.5  # Constants for the step
b1, b2 = 1-a1, 1-a2
t = 0.0

control_inputs = np.deg2rad(np.array([11.74, -0.06, -0.07, 25.4]))
controller_inputs = control_inputs
for step in range(50):

    atmosphere.update(-previous_state["position"][2])
    environment_inputs = {
        "rho": atmosphere.get_density()
    }
    helicopter_data = heli.step(dt, previous_state, controller_inputs, environment_inputs)
    controller_inputs = control_inputs + controller.compute_control_inputs(previous_state, dt, target_state)

    loggerControl.log(
        Time=t,
        TargetYaw=np.rad2deg(target_state["yaw"]), Yaw=np.rad2deg(previous_state["attitude"])[2],
        YawError=np.rad2deg(target_state["yaw"]) - np.rad2deg(previous_state["attitude"])[2],
        TailRotor=np.rad2deg(controller_inputs[3]),

        TargetVz=target_state["vz"], Vz=previous_state["earth_velocity"][2],
        VzError=target_state["vz"] - previous_state["earth_velocity"][2],
        Collective=np.rad2deg(controller_inputs[0]),

        Roll=np.rad2deg(previous_state["attitude"])[0],
        RollError=-3.9 - np.rad2deg(previous_state["attitude"])[0],
        LatCyclic=np.rad2deg(controller_inputs[1]),

        Pitch=np.rad2deg(previous_state["attitude"])[1],
        PitchError=5.12 - np.rad2deg(previous_state["attitude"])[1],
        LongCyclic=np.rad2deg(controller_inputs[2])
    )

    new_state = euler6dof_step(previous_state, helicopter_data["F"], helicopter_data["M"], m, I, dt, a1, b1, a2, b2)
    # freeze_channels(new_state)
    position = new_state["position"]
    body_velocity = new_state["body_velocity"]
    body_acceleration = new_state["body_acceleration"]
    earth_velocity = new_state["earth_velocity"]
    angular_rate = new_state["angular_rate"]
    angular_acceleration = new_state["angular_acceleration"]
    attitude = new_state["attitude"]
    attiude_q = new_state["attitude_q"]

    previous_state = {
        "position": position,
        "body_velocity": body_velocity,
        "earth_velocity": earth_velocity,
        "body_acceleration": body_acceleration,
        "angular_acceleration": angular_acceleration,
        "attitude": attitude,
        "attitude_q": attiude_q,
        "angular_rate": angular_rate,
        "vi_mr_prev": helicopter_data["vi_mr_prev"],
        "gv_7_prev": helicopter_data["gv_7_prev"],
        "gv_8_prev": helicopter_data["gv_8_prev"],
        "gr_7_prev": helicopter_data["gr_7_prev"],
        "gr_8_prev": helicopter_data["gr_8_prev"],
        "vi_tr_prev": helicopter_data["vi_tr_prev"]
    }

    logger.log(Time=t, Altitude=position[2],
            VerticalSpeed=earth_velocity[2],
            Acceleration=body_acceleration[2],
            XPos=position[0],
            YPos=position[1],
            ZPos=position[2],
            EulerX=attitude[0],
            EulerY=attitude[1],
            EulerZ=attitude[2])

    loggerStates.log(Time=t, AccX=body_acceleration[0], AccY=body_acceleration[1], AccZ=body_acceleration[2],
     VelX=earth_velocity[0], VelY=earth_velocity[1], VelZ=earth_velocity[2],
     Roll=np.rad2deg(previous_state["attitude"])[0], Pitch=np.rad2deg(previous_state["attitude"])[1])
    loggerForcesAndMoments.log(Time=t,ForceX=helicopter_data["F"][0], ForceY=helicopter_data["F"][1], ForceZ=helicopter_data["F"][2],
     MomentX=helicopter_data["M"][0], MomentY=helicopter_data["M"][1], MomentZ=helicopter_data["M"][2])
    loggerRotor.log(Time=t, RotorForceX=helicopter_data["RotorForceX"], RotorForceY=helicopter_data["RotorForceY"],
     RotorForceZ=helicopter_data["RotorForceZ"], RotorMomentX=helicopter_data["RotorMomentX"], RotorMomentY=helicopter_data["RotorMomentY"], RotorMomentZ=helicopter_data["RotorMomentZ"] )
    t += dt

logger.save("log.csv")
loggerRotor.save("rotorLog.csv")
loggerForcesAndMoments.save("forcesAndMomentsLog.csv")
loggerControl.save("controllerLog.csv")
loggerStates.save("statesLog.csv")