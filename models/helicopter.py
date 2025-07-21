from models.rotor import Rotor
from config.params import params

class Helicopter:
    def __init__(self):
        self.mass = params["mass"]
        self.rotor = Rotor(params["main_rotor"])
        self.z = 0.0  # vertical position

    def step(self, dt, CT):
        thrust = self.rotor.get_thrust(CT)
        accel = (thrust - self.mass * 9.81) / self.mass
        self.z += accel * dt
        return self.z, accel
