from models.component import Component
import math
import numpy as np

class Fuselage(Component):
    def __init__(self, config):
        super().__init__("Fuselage", config)
        self.fs_fus = config["fuselage"]["FS.FUS"]
        self.wl_fus = config["fuselage"]["WL.FUS"]
        self.xuu_fus = config["fuselage"]["XUU.FUS"]
        self.yvv_fus = config["fuselage"]["YVV.FUS"]
        self.zww_fus = config["fuselage"]["ZWW.FUS"]
        self.fs_cg = config["FS.CG"]
        self.wl_cg = config["WL.CG"]
        self.interactions_mr = config["interactions_mr"]
        self.fs_hub = config["main_rotor"]["FS.HUB"]
        self.wl_hub = config["main_rotor"]["WL.HUB"]
        self.rpm_mr = config["main_rotor"]["RPM.MR"]
        self.h_hub = (self.wl_hub - self.wl_cg)/12.0
        self.d_hub = (self.fs_hub - self.fs_cg)/12.0
        self.h_fus = (self.wl_fus - self.wl_cg)/12.0
        self.d_fus = (self.fs_fus - self.fs_cg)/12.0

    def get_force_and_moment(self, state, controls, environment):
        airspeed = state.get("airspeed", np.array([0.0, 0.0, 0.0]))
        rho = environment.get("rho", 0.0023769)
        omega_mr = self.rpm_mr*2.0*math.pi/60.0
        vi_mr = state.get("vi_mr_prev", 0.0)
        wa_fus = airspeed[2]-float(self.interactions_mr)*vi_mr
        if wa_fus==0:
            wa_fus=0.0001

        d_fw = (airspeed[0]/(-wa_fus)*(self.h_hub-self.h_fus))-(self.d_fus-self.d_hub)
        d_fw = 3.0*d_fw
        
        x_fus = rho/2.0*self.xuu_fus*abs(airspeed[0])*airspeed[0]
        y_fus = rho/2.0*self.yvv_fus*abs(airspeed[1])*airspeed[1]
        z_fus = rho/2.0*self.zww_fus*abs(wa_fus)*wa_fus
        l_fus = y_fus*self.h_fus
        m_fus = z_fus*d_fw - x_fus*self.h_fus

        p_parasite = - x_fus * airspeed[0] - y_fus * airspeed[1] - z_fus*wa_fus

        return {"Fx": x_fus, "Fy": y_fus, "Fz": z_fus, "Mx": l_fus, "My": m_fus, "Mz": p_parasite/omega_mr, "p_parasite": p_parasite}
