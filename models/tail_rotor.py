from models.component import Component
import math
import numpy as np

class TailRotor(Component):
    def __init__(self, config):
        super().__init__("TailRotor", config)
        self.fs_tr = config["tail_rotor"]["FS.TR"]
        self.wl_tr = config["tail_rotor"]["WL.TR"]
        self.fs_cg = config["FS.CG"]
        self.wl_cg = config["WL.CG"]
        self.interactions = config["interactions_tr"]
        self.rpm_tr = config["tail_rotor"]["RPM.TR"]
        self.r_tr = config["tail_rotor"]["R.TR"]
        self.a_tr = config["tail_rotor"]["A.TR"]
        self.sol_tr = config["tail_rotor"]["SOL.TR"]
        self.twst_tr = config["tail_rotor"]["TWST.TR"]
        self.h_tr = (self.wl_tr - self.wl_cg)/12.0
        self.d_tr = (self.fs_tr - self.fs_cg)/12.0
        self.omega_tr = (self.rpm_tr)*2.0*math.pi/60.0

    def get_force_and_moment(self, state, controls, environment):
        airspeed = state.get("body_velocity", np.array([0.0, 0.0, 0.0]))
        angular_rate = state.get("angular_rate", np.array([0.0, 0.0, 0.0]))
        rho = environment.get("rho", 0.0023769)
        vr_tr = -(airspeed[1]-angular_rate[2]*self.d_tr + angular_rate[0]*self.h_tr)
        vb_tr = vr_tr + 2.0/3.0*self.omega_tr*self.r_tr*(controls[3]+self.twst_tr*0.75)
        vi_tr = state.get("vi_tr_prev", 0.0)

        tolerance_tr = 0.01

        vb1 = 50.63
        kb1 = 0.76
        solve_gain_tr = 0.1

        for i in range(100):
            if abs(airspeed[0]<=vb1) and self.interactions:
                kb11 = ((1.0-kb1)*airspeed[0]**2.0/(vb1**2.0)+kb1)
            else:
                kb11 = 1.0

            thrust_tr = kb11*(vb_tr-vi_tr)*self.omega_tr*self.r_tr*rho*self.a_tr*self.sol_tr*math.pi*self.r_tr*self.r_tr/4.0
            vprime_tr = np.sqrt(airspeed[0]**2.0+airspeed[2]**2.0+(vr_tr-vi_tr)**2.0)
            trvind_2 = (rho*math.pi*self.r_tr**2.0)

            if(abs(vprime_tr)>0.01):
                vi_tr_new = 0.5*thrust_tr/(trvind_2*vprime_tr)
            else:
                vi_tr_new = 0.1
                thrust_tr = 0.1

            correction_tr = solve_gain_tr*(vi_tr_new-vi_tr)
            vi_tr = vi_tr + correction_tr

        power_tr = thrust_tr*vi_tr
        y_tr = thrust_tr
        l_tr = y_tr*self.h_tr
        n_tr = -y_tr*self.d_tr

        return {"Fx": 0.0, "Fy": y_tr, "Fz": 0.0, "Mx": l_tr, "My": 0.0, "Mz": n_tr, "Power": power_tr, "vi_tr": vi_tr}
