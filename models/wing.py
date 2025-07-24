from models.component import Component
import math

class Wing(Component):
    def __init__(self, config):
        super().__init__("Wing", config)
        self.fs_wn = config["wing"]["FS.WN"]
        self.wl_wn = config["wing"]["WL.WN"]
        self.zuu_wn = config["wing"]["ZUU.WN"]
        self.zuw_wn = config["wing"]["ZUW.WN"]
        self.zmax_wn = config["wing"]["ZMAX.WN"]
        self.b = config["wing"]["B.WN"]
        self.fs_cg = config["FS.CG"]
        self.wl_cg = config["WL.CG"]
        self.h_wn = (self.wl_wn - self.wl_hg)/12.0
        self.d_wn = (self.fs_wn - self.fs_cg)/12.0

    def get_force_and_moment(self, state, controls, environment):
        airspeed = state.get("airspeed", math.array([0.0, 0.0, 0.0]))
        angular_rate = state.get("angular_rate", math.array([0.0, 0.0, 0.0]))
        rho = environment.get("rho", 0.0023769)
        vi_mr = state.get("vi_mr_prev", 0.0)

        wa_wn = airspeed(2)-vi_mr
        vta_wn = math.sqrt(airspeed(0)**2.0 +wa_wn**2.0)

        z_wn = rho/2.0*(self.zuu_wn*airspeed(0)**2.0 + self.zuw_wn*airspeed(0)*wa_wn)
        x_wn =-rho/2.0/math.pi/vta_wn/vta_wn*(self.zuu_wn*airspeed(0)**2.0 + self.zuw_wn*airspeed(0)*wa_wn)**2.0

        if abs(wa_wn) > 0.3 * abs(airspeed(0)):
            z_wn = rho/2.0*self.zmax_wn*abs(vta_wn)*wa_wn

        power_wn = abs(x_wn*airspeed(0))
        return {"Fx": 0.0, "Fy": 0.0, "Fz": 0.0, "Mx": 0.0, "My": m_ht, "Mz": 0.0}
