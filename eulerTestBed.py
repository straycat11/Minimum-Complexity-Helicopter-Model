import numpy as np
import matplotlib.pyplot as plt
from models.integrator import euler6dof_step
from scipy.spatial.transform import Rotation as R

# Initial state
state = {
    "position": np.array([0.0, 0.0, 0.0]),
    "body_velocity": np.array([0.0, 0.0, 0.0]),
    "earth_velocity": np.array([0.0, 0.0, 0.0]),
    "body_acceleration": np.array([0.0, 0.0, 0.0]),
    "angular_rate": np.array([0.0, 0.0, 0.0]),
    "angular_acceleration": np.array([0.0, 0.0, 0.0]),
    "attitude": np.array([0.0, 0.0, 0.0]),  # Euler angles
    "attitude_q": np.array([0.0, 0.0, 0.0, 1.0])  # Quaternion [x, y, z, w]
}

# Constants
dt = 0.01  # seconds
T = 5.0    # total simulation time
steps = int(T / dt)

m = 5401.0 / 32.2  # slug
I = np.diag([500.0, 600.0, 700.0])  # Inertia matrix
I[0, 2] = I[2, 0] = 50.0  # Cross product inertia

# Integration scheme constants (e.g., Adams-Bashforth or similar)
a1, b1 = 1.0, 0.0
a2, b2 = 1.0, 0.0

# Force and moment: Apply small pitch moment
F = -np.array([0.0, 0.0, 0.0])  
M = np.array([0.0, 0.0, -10000.0])  

# Logging arrays
positions = []
angles = []
angular_rates = []

for _ in range(steps):
    state = euler6dof_step(state, F, M, m, I, dt, a1, b1, a2, b2)
    positions.append(state["position"])
    angles.append(state["attitude"])
    angular_rates.append(state["angular_rate"])

# Convert to arrays
positions = np.array(positions)
angles = np.array(angles)
angular_rates = np.array(angular_rates)

# Plotting
time = np.linspace(0, T, steps)

plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(time, positions)
plt.title("Position (ft)")
plt.legend(["x", "y", "z"])
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(time, angles * 180/np.pi)  # Convert rad to deg
plt.title("Euler Angles (deg)")
plt.legend(["Roll", "Pitch", "Yaw"])
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(time, angular_rates)
plt.title("Angular Rates (rad/s)")
plt.legend(["p", "q", "r"])
plt.grid()

plt.tight_layout()
plt.show()
