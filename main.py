import time
import numpy as np
# from models.helicopter import Helicopter
from models.integrator import euler6dof_step
from utils.logger import Logger
from utils.plotter import plot_variables

# heli = Helicopter()
logger = Logger(variable_names=["Time", "Altitude", "VerticalSpeed", "Acceleration", "XPos", "YPos", "EulerX", "EulerY", "EulerZ"])

# Define initial state and parameters (these are just placeholders)
previous_state = {
    "position": [0.0, 0.0, 0.0],
    "body_velocity": [0.0, 0.0, 0.0],
    "earth_velocity": [0.0, 0.0, 0.0],
    "body_acceleration": [0.0, 0.0, 0.0],
    "angular_acceleration": [0.0, 0.0, 0.0],
    "attitude": [0.0, 0.0, 0.0],
    "angular_rate": [0.0, 0.0, 0.0]
}

# Define parameters (replace with actual values)
F = np.array([0.0, 0.0, 10.0])  # lb
M = np.array([0.0, 1000.0, 0.0])  # lb
m = 9000.0  # lb
I = np.array([[2593.0, 0.0, 0.0], [0.0, 14320.0, 0.0], [0.0, 0.0, 12330.0]])  # Moment of inertia matrix
dt = 0.01  # Time step
a1, a2 = 1.5, 0.5  # Constants for the step
b1, b2 = 1-a1, 1-a2
t = 0.0

for step in range(1000):

    new_state = euler6dof_step(previous_state, F, M, m, I, dt, a1, b1, a2, b2)

    position = new_state["position"]
    body_velocity = new_state["body_velocity"]
    body_acceleration = new_state["body_acceleration"]
    earth_velocity = new_state["earth_velocity"]
    angular_rate = new_state["angular_rate"]
    angular_acceleration = new_state["angular_acceleration"]
    attitude = new_state["attitude"]

    previous_state = {
        "position": position,
        "body_velocity": body_velocity,
        "earth_velocity": earth_velocity,
        "body_acceleration": body_acceleration,
        "angular_acceleration": angular_acceleration,
        "attitude": attitude,
        "angular_rate": angular_rate
    }

    logger.log(Time=t, Altitude=position[2],
                VerticalSpeed=earth_velocity[2],
                  Acceleration=body_acceleration[2],
                    XPos=position[0],
                      YPos=position[1],
                      EulerX=attitude[0],
                        EulerY=attitude[1],
                          EulerZ=attitude[2])
    t += dt

logger.save("log.csv")

plot_variables("log.csv", 
               x_var="Time", 
               y_vars=["Altitude", "VerticalSpeed", "Acceleration","XPos","YPos", "EulerX", "EulerY", "EulerZ"],
               title_prefix="Helicopter", 
               ylabel="Flight Variable")

