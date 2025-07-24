import numpy as np
from models.integrator import euler6dof_step
from models.fuselage import Fuselage
from models.htail import HorizontalTail
from models.vtail import VerticalTail
from models.rotor import Rotor
from models.tail_rotor import TailRotor
from models.wing import Wing

def sum_force_moment_dicts(dicts):
    valid_keys = {"Fx", "Fy", "Fz", "Mx", "My", "Mz"}
    total = {key: 0.0 for key in valid_keys}
    
    for d in dicts:
        for key in valid_keys:
            if key in d:
                total[key] += d[key]
    
    return total

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
        self.fuselage = Fuselage(params)
        self.h_tail = HorizontalTail(params)
        self.v_tail = VerticalTail(params)
        self.wing = Wing(params)
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
        f_fuselage = self.fuselage.get_force_and_moment(states,control_inputs,environment_inputs)
        f_htail = self.h_tail.get_force_and_moment(states,control_inputs,environment_inputs)
        f_vtail = self.v_tail.get_force_and_moment(states,control_inputs,environment_inputs)
        f_wing = self.wing.get_force_and_moment(states,control_inputs,environment_inputs)

        components_forces = [
            f_main_rotor, f_tail_rotor, f_fuselage,
            f_htail, f_vtail, f_wing
        ]
        total_force_moment = sum_force_moment_dicts(components_forces)
        
        F = np.array([total_force_moment["Fx"],total_force_moment["Fy"],total_force_moment["Fz"]])
        M = np.array([total_force_moment["Mx"],total_force_moment["My"],total_force_moment["Mz"]])
        return {"F": F, "M": M, "vi_mr_prev": f_main_rotor["vi_mr"], "gv_7_prev": f_main_rotor["gv_7"],
            "gv_8_prev": f_main_rotor["gv_8"], "gr_7_prev": f_main_rotor["gr_7"], "gr_8_prev": f_main_rotor["gr_8"],
            "power_mr": f_main_rotor["power_mr"], "torque_mr": f_main_rotor["torque_mr"], "vi_tr_prev": f_tail_rotor["vi_tr"]}
        

