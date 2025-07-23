import numpy as np
from models.integrator import euler6dof_step
from models.fuselage import Fuselage
from models.htail import HorizontalTail
from models.vtail import VerticalTail
from models.rotor import Rotor
from models.tail_rotor import TailRotor
from models.wing import Wing

class Helicopter:
    def __init__(self, params):
        self.mass = params["WT"]
        self.I = np.diag([params["Ix"], params["Iy"], params["Iz"]])
        self.data = {
            "position": [0.0, 0.0, 0.0],
            "body_velocity": [0.0, 0.0, 0.0],
            "earth_velocity": [0.0, 0.0, 0.0],
            "body_acceleration": [0.0, 0.0, 0.0],
            "angular_acceleration": [0.0, 0.0, 0.0],
            "attitude": [0.0, 0.0, 0.0],
            "angular_rate": [0.0, 0.0, 0.0]
            }

        self.rotor = Rotor(params)
        # self.components = []
        # self.components.append(Fuselage(params))
        # self.components.append(HorizontalTail(params))
        # self.components.append(VerticalTail(params))
        # self.components.append(Rotor(params))
        # self.components.append(TailRotor(params))
        # self.components.append(Wing(params))

    def step(self, dt, states, control_inputs, environment_inputs):

        # Sum all forces/moments
        total_force = np.zeros(3)
        total_moment = np.zeros(3)
        # for c in self.components:
        #     f = c.get_force_and_moment(self.state, control_inputs, environment_inputs)
        #     total_force += np.array([f["Fx"], f["Fy"], f["Fz"]])
        #     total_moment += np.array([f["Mx"], f["My"], f["Mz"]])

        f = self.rotor.get_force_and_moment(states,control_inputs,environment_inputs)
        vi_mr_prev = f["vi_mr"]
        gv_7_prev = f["gv_7"]
        gv_8_prev = f["gv_8"]
        gr_7_prev = f["gr_7"]
        gr_8_prev = f["gr_8"]

        F = np.array([f["Fx"],f["Fy"],f["Fz"]])
        M = np.array([f["Mx"],f["My"],f["Mz"]])
        return {"F": F, "M": M, "vi_mr_prev": vi_mr_prev, "gv_7_prev": gv_7_prev, "gv_8_prev": gv_8_prev, "gr_7_prev": gr_7_prev, "gr_8_prev": gr_8_prev}
        
