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
        self.tail_rotor = TailRotor(params)
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

        f_main_rotor = self.rotor.get_force_and_moment(states,control_inputs,environment_inputs)
        f_tail_rotor = self.tail_rotor.get_force_and_moment(states,control_inputs,environment_inputs)

        F = np.array([f_main_rotor["Fx"],f_main_rotor["Fy"],f_main_rotor["Fz"]])
        M = np.array([f_main_rotor["Mx"],f_main_rotor["My"],f_main_rotor["Mz"]])
        return {"F": F, "M": M, "vi_mr_prev": f_main_rotor["vi_mr"], "gv_7_prev": f_main_rotor["gv_7"],
            "gv_8_prev": f_main_rotor["gv_8"], "gr_7_prev": f_main_rotor["gr_7"], "gr_8_prev": f_main_rotor["gr_8"],
            "power_mr": f_main_rotor["power_mr"], "torque_mr": f_main_rotor["torque_mr"], "vi_tr_prev": f_tail_rotor["vi_tr"]}
        
