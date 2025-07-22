import numpy as np
from models.integrator import euler6dof_step

class Helicopter:
    def __init__(self, params):
        self.mass = params["mass"]
        self.I = np.diag([params["Ix"], params["Iy"], params["Iz"]])
        self.state = {
            "position": np.zeros(3),
            "velocity": np.zeros(3),
            "attitude": np.zeros(3),  # Euler: phi, theta, psi
            "angular_rate": np.zeros(3),
        }

        self.components = [...]  # Rotor, fuselage, tail, etc.

    def step(self, dt, control_inputs, environment_inputs):
        # Build full flight state
        state_input = self.state | control_inputs | environment_inputs # merge dictionaries

        # Sum all forces/moments
        total_force = np.zeros(3)
        total_moment = np.zeros(3)
        for c in self.components:
            f = c.get_force(state_input)
            m = c.get_moment(state_input) if hasattr(c, "get_moment") else np.zeros(3)
            total_force += np.array([f["Fx"], f["Fy"], f["Fz"]])
            total_moment += m

        # Propagate 6-DOF motion
        self.state = euler6dof_step(self.state, total_force, total_moment, self.mass, self.I, dt)
