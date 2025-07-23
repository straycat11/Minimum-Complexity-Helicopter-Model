from models.component import Component
import math
import numpy as np

class HorizontalTail(Component):
    def __init__(self, config):
        super().__init__("HorizontalTail", config)
        self.fs_ht = config["horizontal_tail"]["FS.HT"]
        self.wl_ht = config["horizontal_tail"]["WL.HT"]
        self.zuu_ht = config["horizontal_tail"]["ZUU.HT"]
        self.zuw_ht = config["horizontal_tail"]["ZUW.HT"]
        self.zmax_ht = config["horizontal_tail"]["ZMAX.HT"]
        self.fs_hub = config["main_rotor"]["FS.HUB"]
        self.wl_hub = config["main_rotor"]["WL.HUB"]
        self.r_mr = config["main_rotor"]["R.MR"]
        self.fs_cg = config["FS.CG"]
        self.wl_cg = config["WL.CG"]
        self.h_ht = (self.wl_ht - self.wl_cg)/12.0
        self.d_ht = (self.fs_ht - self.fs_cg)/12.0
        self.h_hub = (self.wl_hub - self.wl_cg)/12.0
        self.d_hub = (self.fs_hub - self.fs_cg)/12.0


    def get_force_and_moment(self, state, controls, environment):
        airspeed = state.get("airspeed", np.array([0.0, 0.0, 0.0]))
        angular_rate = state.get("angular_rate", np.array([0.0, 0.0, 0.0]))
        rho = environment.get("rho", 1.225)
        vi_mr = state.get("vi_mr_prev", 0.0)
        vi_tr = state.get("vi_tr_prev", 0.0)

        d_dw = (airspeed[0]/(vi_mr-airspeed[2])*(self.h_hub-self.h_ht))-(self.d_ht-self.d_hub-self.r_mr)
        d_dw = d_dw + 1.0
        eps_ht = 0.5*(1.0 + math.copysign(1.0, d_dw)) 
        if d_dw > 0 and d_dw < self.r_mr:
            eps_ht = 2.0 - (1.0 - d_dw/self.r_mr)
        else:
            eps_ht = 0.0

        wa_ht = airspeed[2] - eps_ht*vi_mr + self.d_ht*angular_rate[1]
        vta_ht = math.sqrt(airspeed[0]*airspeed[0]+airspeed[1]*airspeed[1] + wa_ht*wa_ht)
        z_ht = rho/2*(self.zuu_ht*abs(airspeed[0])*airspeed[0]+self.zuw_ht*abs(airspeed[0])*wa_ht)

        if abs(wa_ht) > 0.3*abs(airspeed[0]):
            z_ht = rho/2.0*self.zmax_ht*abs(vta_ht)*wa_ht

        m_ht = z_ht * self.d_ht
        return {"Fx": 0.0, "Fy": 0.0, "Fz": 0.0, "Mx": 0.0, "My": m_ht, "Mz": 0.0}
