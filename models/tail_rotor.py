from models.component import Component
import math

class TailRotor(Component):
    def __init__(self, config):
        super().__init__("TailRotor", config)
        self.fs_tr = config["tail_rotor"]["FS.TR"]
        self.wl_tr = config["tail_rotor"]["WL.TR"]
        self.fs_cg = config["FS.CG"]
        self.wl_cg = config["WL.CG"]
        self.rpm_tr = config["tail_rotor"]["RPM.TR"]
        self.r_tr = config["tail_rotor"]["R.TR"]
        self.a_tr = config["tail_rotor"]["A.TR"]
        self.sol_tr = config["tail_rotor"]["SOL.TR"]
        self.twst_tr = config["tail_rotor"]["TWST.TR"]
        self.h_tr = (self.wl_tr - self.wl_cg)/12.0
        self.d_tr = (self.fs_tr - self.fs_cg)/12.0
        self.omega_tr = (self.rpm_tr)*2.0*math.pi/60.0

    def get_force_and_moment(self, state, controls, environment):
        airspeed = state.get("airspeed", math.array([0.0, 0.0, 0.0]))
        angular_rate = state.get("angular_rate", math.array([0.0, 0.0, 0.0]))
        rho = environment.get("rho", 1.225)
        vr_tr = -(airspeed(1)-angular_rate(2)*self.d_tr + angular_rate(0)*self.h_tr)
        vb_tr = vr_tr + 2.0/3.0*self.omega_tr*self.r_tr*(controls(3)+self.twst_tr*0.75)
        vi_tr = state.get("vi_tr_prev", 0.0)

        for i in range(4):
            thrust_tr = (vb_tr-vi_tr)*self.omega_tr*self.r_tr*rho*self.a_tr*self.sol_tr*math.pi*self.r_tr*self.r_tr/4.0
            vhat_2 = (airspeed(2)+angular_rate(1)*self.d_tr)**2.0 + airspeed(0)**2.0 + vr_tr*(vr_tr-2.0*vi_tr)
            vi_tr_2 = math.sqrt((vhat_2/2.0)*(vhat_2/2.0)+(thrust_tr/2.0/(rho*math.pi*self.r_tr**2.0))**2.0)-vhat_2/2.0
            vi_tr = math.sqrt(math.abs(vi_tr_2))
        
        power_tr = thrust_tr*vi_tr
        y_tr = thrust_tr
        l_tr = y_tr*self.h_tr
        n_tr = -y_tr*self.d_tr
        return {"Fx": 0.0, "Fy": y_tr, "Fz": 0.0, "Mx": l_tr, "My": 0.0, "Mz": n_tr, "Power": power_tr, "vi_tr": vi_tr}
