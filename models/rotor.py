from models.component import Component
import math
import numpy as np

class Rotor(Component):
    def __init__(self, config):
        super().__init__("MainRotor", config)
        self.fs_hub = config["main_rotor"]["FS.HUB"]
        self.wl_hub = config["main_rotor"]["WL.HUB"]
        self.r_mr = config["main_rotor"]["R.MR"]
        self.a_mr = config["main_rotor"]["A.MR"]
        self.b_mr = config["main_rotor"]["B.MR"]
        self.c_mr = config["main_rotor"]["C.MR"]
        self.e_mr = config["main_rotor"]["E.MR"]
        self.i_b = config["main_rotor"]["IB"]
        self.k1 = config["main_rotor"]["K1"]
        self.cd0 = config["main_rotor"]["CD0"]
        self.rpm_mr = config["main_rotor"]["RPM.MR"]
        self.twst_mr = config["main_rotor"]["TWST.MR"]
        self.main_rotor_incidence = config["main_rotor"]["IS"]
        self.wt = config["WT"]
        self.fs_cg = config["FS.CG"]
        self.wl_cg = config["WL.CG"]
        self.dt = config["dt"]
        self.a2 = config["a2"]
        self.b2 = 1.0-self.a2
        self.h_hub = (self.wl_hub - self.wl_cg)/12.0
        self.d_hub = (self.fs_hub - self.fs_cg)/12.0
        self.vtrans = 50.0
        self.rho_sea_level = config["rho_sea_level"]

    def get_force_and_moment(self, state, controls, environment):
        airspeed = state.get("body_velocity", np.array([0.0, 0.0, 0.0]))
        angular_rate = state.get("angular_rate", np.array([0.0, 0.0, 0.0]))
        velocity_dot = state.get("earth_velocity", np.array([0.0, 0.0, 0.0]))
        rho = environment.get("rho", 0.0023769)
        omega_mr = self.rpm_mr*2.0*math.pi/60.0
        v_tip = self.r_mr*omega_mr
        fr_mr = self.cd0*self.r_mr*self.b_mr*self.c_mr
        gam_om_16 = rho*self.a_mr*self.c_mr*self.r_mr**4.0/self.i_b*omega_mr/16.0*(1.0+8.0/3.0*self.e_mr/self.r_mr)
        kc = (0.75*omega_mr*self.e_mr/self.r_mr/gam_om_16)+self.k1
        itb2_om = omega_mr/(1.0+(omega_mr/gam_om_16)**2.0)
        itb = itb2_om*omega_mr/gam_om_16
        dl_db1 = self.b_mr/2.0*(1.5*self.i_b*self.e_mr/self.r_mr*omega_mr*omega_mr)
        dl_da1 = rho/2.0*self.a_mr*self.b_mr*self.c_mr*self.r_mr*v_tip*v_tip*self.e_mr/6.0
        ct = self.wt/(rho*math.pi*self.r_mr*self.r_mr*v_tip*v_tip)
        a_sigma = self.a_mr*self.b_mr*self.c_mr/self.r_mr/math.pi
        db1dv = 2.0/omega_mr/self.r_mr*(8.0*ct/a_sigma+(math.sqrt(ct/2.0)))
        da1du = -db1dv
        vi_mr = state.get("vi_mr_prev", 0.0)
        gv_7_prev = state.get("gv_7_prev", 0.0)
        gv_8_prev = state.get("gv_8_prev", 0.0)
        gr_7_prev = state.get("gr_7_prev", 0.0)
        gr_8_prev = state.get("gr_8_prev", 0.0)

        if airspeed[0] < self.vtrans:
            wake_fm = 1.0
        else:
            wake_fm = 0.0

        a_sum = gv_8_prev-controls[1]+kc*gv_7_prev+db1dv*airspeed[1]*(1.0+wake_fm)
        b_sum = gv_7_prev+controls[2]-kc*gv_8_prev+da1du*airspeed[0]*(1.0+2.0*wake_fm)

        gr_7 = -itb*b_sum - itb2_om*a_sum-angular_rate[1]
        gr_8 = -itb*a_sum + itb2_om*b_sum-angular_rate[0]

        gv_7 = gv_7_prev + self.dt*(self.a2*gr_7 + self.b2 * gr_7_prev)
        gv_8 = gv_8_prev + self.dt*(self.a2*gr_8 + self.b2 * gr_8_prev)

        wr = airspeed[2]+(gv_7-self.main_rotor_incidence)*airspeed[0]-gv_8*airspeed[1]
        wb = wr + (2.0/3.0)*omega_mr*self.r_mr*(controls[0]+0.75*self.twst_mr)

        for i in range(25):
            thrust_mr = (wb-vi_mr)*omega_mr*self.r_mr*rho*self.a_mr*self.b_mr*self.c_mr*self.r_mr/4.0
            vhat_2 = airspeed[0]**2.0 + airspeed[1]**2.0 + wr*(wr-2.0*vi_mr)
            vi_mr_2 = math.sqrt((vhat_2/2.0)*(vhat_2/2.0)+(thrust_mr/2.0/(rho*math.pi*self.r_mr**2.0))**2.0)-vhat_2/2.0
            vi_mr = math.sqrt(abs(vi_mr_2))

        p_induced_mr = thrust_mr*vi_mr
        p_climb = self.wt*(-velocity_dot[2])
        p_profile_mr = rho/2.0*(fr_mr/4.0)*omega_mr*self.r_mr*(omega_mr**2.0*self.r_mr**2.0 + 4.6 * (airspeed[0]**2.0+airspeed[1]**2.0))
        power_mr = p_induced_mr + p_climb + p_profile_mr
        power_rotor_mr = p_induced_mr + p_profile_mr
        torque_mr = power_mr/omega_mr

        x_mr = -thrust_mr*(gv_7-self.main_rotor_incidence)
        y_mr = thrust_mr*gv_8
        z_mr = -thrust_mr
        l_mr = y_mr*self.h_hub + dl_db1*gv_8+ dl_da1*(gv_7+controls[2]-self.k1*gv_8)
        m_mr = z_mr*self.d_hub - x_mr*self.h_hub + dl_db1*gv_7 + dl_da1*(-gv_8+controls[1]-self.k1*gv_7)
        n_mr = torque_mr

        return {"Fx": x_mr, "Fy": y_mr, "Fz": z_mr, "Mx": l_mr, "My": m_mr, "Mz": n_mr, "vi_mr": vi_mr, "gv_7": gv_7, "gv_8": gv_8, "gr_7": gr_7, "gr_8": gr_8, "power_mr": power_mr, "torque_mr": torque_mr}
