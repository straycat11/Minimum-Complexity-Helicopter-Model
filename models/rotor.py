import numpy as np

def compute_lift(thrust_coeff, rho, A, omega):
    return thrust_coeff * rho * A * (omega**2)

class Rotor:
    def __init__(self, config):
        self.radius = config["radius"]
        self.omega = config["omega"]
        self.A = np.pi * self.radius ** 2

    def get_thrust(self, CT, rho=1.225):
        return compute_lift(CT, rho, self.A, self.omega)
