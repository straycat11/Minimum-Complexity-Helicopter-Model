import time
import numpy as np
from models.helicopter import Helicopter
from models.atmosphere import Atmosphere
from models.integrator import euler6dof_step
from utils.logger import Logger
from utils.plotter import plot_variables
from config.params import params

atmosphere = Atmosphere() 
logger = Logger(variable_names=["Time", "Altitude", "VerticalSpeed", "Acceleration", "XPos", "YPos", "ZPos", "EulerX", "EulerY", "EulerZ"])
# loggerRotor = Logger(variable_names=["Time", "RotorForceX", "RotorForceY", "RotorForceZ", "RotorMomentZ", "RotorTorque"])
loggerForcesAndMoments = Logger(variable_names=["Time", "ForceX", "ForceY", "ForceZ", "MomentX", "MomentY", "MomentZ"])

# Define initial state and parameters (these are just placeholders)
previous_state = {
    "position": [0.0, 0.0, 0.0],
    "body_velocity": [0.0, 0.0, 0.0],
    "earth_velocity": [0.0, 0.0, 0.0],
    "body_acceleration": [0.0, 0.0, 0.0],
    "angular_acceleration": [0.0, 0.0, 0.0],
    "attitude": [0.0, 0.0, 0.0],
    "attitude_q": [1.0, 0.0, 0.0, 0.0],
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
m = 9000.0  # lb
I = np.array([[2593.0, 0.0, 0.0], [0.0, 14320.0, 0.0], [0.0, 0.0, 12330.0]])  # Moment of inertia matrix
dt = 0.01  # Time step
a1, a2 = 1.5, 0.5  # Constants for the step
b1, b2 = 1-a1, 1-a2
t = 0.0

control_inputs = np.deg2rad(np.array([11.6, 0.0, 0.0, 20.0]))

for step in range(50):

    atmosphere.update(-previous_state["position"][2])
    environment_inputs = {
        "rho": atmosphere.get_density()
    }
    helicopter_data = heli.step(dt, previous_state, control_inputs, environment_inputs)

    new_state = euler6dof_step(previous_state, helicopter_data["F"], helicopter_data["M"], m, I, dt, a1, b1, a2, b2)
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

    
    loggerForcesAndMoments.log(Time=t,ForceX=helicopter_data["F"][0], ForceY=helicopter_data["F"][1], ForceZ=helicopter_data["F"][2],
     MomentX=helicopter_data["M"][0], MomentY=helicopter_data["M"][1], MomentZ=helicopter_data["M"][2])
    t += dt

logger.save("log.csv")
# loggerRotor.save("rotorLog.csv")
loggerForcesAndMoments.save("forcesAndMomentsLog.csv")

plot_variables("log.csv", 
               x_var="Time", 
               y_vars=["Altitude", "VerticalSpeed", "Acceleration","XPos","YPos", "EulerX", "EulerY", "EulerZ"],
               title_prefix="Helicopter", 
               ylabel="Flight Variable")

# plot_variables("rotorLog.csv", 
#                x_var="Time", 
#                y_vars=["RotorForceX", "RotorForceY", "RotorForceZ", "RotorMomentZ", "RotorTorque"],
#                title_prefix="Helicopter", 
#                ylabel="Flight Variable")

# plot_variables("forcesAndMomentsLog.csv", 
#                x_var="Time", 
#                y_vars=["ForceX", "ForceY", "ForceZ", "MomentX", "MomentY", "MomentZ"],
#                title_prefix="Helicopter", 
#                ylabel="Flight Variable")
