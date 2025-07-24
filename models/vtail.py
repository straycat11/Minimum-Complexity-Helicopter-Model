from models.component import Component
import math

class VerticalTail(Component):
    def __init__(self, config):
        super().__init__("VerticalTail", config)
        self.fs_vt = config["vertical_tail"]["FS.VT"]
        self.wl_vt = config["vertical_tail"]["WL.VT"]
        self.yuu_vt = config["vertical_tail"]["YUU.VT"]
        self.yuv_vt = config["vertical_tail"]["YUV.VT"]
        self.ymax_vt = config["vertical_tail"]["YMAX.VT"]
        self.fs_cg = config["FS.CG"]
        self.wl_cg = config["WL.CG"]
        self.h_vt = (self.wl_vt - self.wl_cg)/12.0
        self.d_vt = (self.fs_vt - self.fs_cg)/12.0


    def get_force_and_moment(self, state, controls, environment):
        airspeed = state.get("airspeed", math.array([0.0, 0.0, 0.0]))
        angular_rate = state.get("angular_rate", math.array([0.0, 0.0, 0.0]))
        rho = environment.get("rho", 0.0023769)
        vi_tr = state.get("vi_tr_prev", 0.0)
        va_vt = airspeed(1)+vi_tr-self.d_vt*angular_rate(2)
        vta_vt = math.sqrt(airspeed(0)**2+va_vt**2)
        y_vt = rho/2*(self.yuu_vt+abs(airspeed(0))*airspeed(0)+self.yuv_vt+abs(airspeed(0))*va_vt)

        if abs(va_vt) > 0.3*abs(airspeed(0)):
            y_vt = rho/2*self.ymax_vt*abs(vta_vt)*va_vt

        l_vt = y_vt * self.h_vt
        n_vt = -y_vt * self.d_vt
        return {"Fx": 0.0, "Fy": 0.0, "Fz": 0.0, "Mx": l_vt, "My": 0.0, "Mz": n_vt}
