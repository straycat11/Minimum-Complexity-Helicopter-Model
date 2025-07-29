from controllers.pid import PID
import numpy as np
def wrap_to_pi(angle):
    """Wrap angle to [-π, π]"""
    return (angle + np.pi) % (2 * np.pi) - np.pi

class HelicopterController:
    def __init__(self):
        self.phi_pid = PID(kp=3.2*0.5, ki=0.0, kd=0.0)
        self.theta_pid = PID(kp=0.6, ki=0.0, kd=0.0)
        self.psi_pid = PID(kp=0.8, ki=0.0, kd=0.0)

        self.vspeed_pid = PID(kp=0.01, ki=0.005, kd=0.002)
        self.x_speed_pid = PID(kp=0.01, ki=0.002, kd=0.001)
        self.y_speed_pid = PID(kp=0.01, ki=0.002, kd=0.001)

        self.x_acc_pid = PID(kp=0.01, ki=0.002, kd=0.001)
        self.y_acc_pid = PID(kp=0.01, ki=0.002, kd=0.001)


    def compute_control_inputs(self, state, dt, targets):
        # Extract states
        roll = state["attitude"][0]     # radians
        pitch = state["attitude"][1]     # radians
        roll_rate = state["angular_rate"][0] # radians/s
        pitch_rate = state["angular_rate"][1] # radians/s
        yaw = state["attitude"][2]      # radians
        vz = state["earth_velocity"][2] # m/s (positive up)
        vx = state["earth_velocity"][0] # m/s (forward)
        vy = state["earth_velocity"][1] # m/s (forward)
        ax = state["body_acceleration"][0] # m/s (forward)
        ay = state["body_acceleration"][1] # m/s (forward)

        # Compute errors
        yaw_err = wrap_to_pi(targets["yaw"] - yaw)
        vz_err = targets["vz"] - vz
        vx_err = targets["vx"] - vx
        vy_err = targets["vy"] - vy

        # Get control corrections
        # y_acc_ref = self.y_speed_pid.update(vy_err, dt)
        # ay_err = y_acc_ref - ay
        # phi_ref = self.x_acc_pid.update(ay_err, dt)
        phi_ref = 0.0
        phi_err = phi_ref - roll
        lateral_cyclic = -self.phi_pid.update(phi_err, dt, roll_rate)

        # tail_rotor = self.psi_pid.update(yaw_err, dt)
        tail_rotor = 0.0
        # collective = self.vspeed_pid.update(vz_err, dt)
        collective = 0.0
        # x_acc_ref = self.x_speed_pid.update(vx_err, dt, ax)
        # ax_err = x_acc_ref - ax
        # theta_ref = self.x_acc_pid.update(ax_err, dt)
        theta_ref = 0.0
        theta_err = theta_ref - pitch
        # longitudinal_cyclic = self.theta_pid.update(theta_err, dt, pitch_rate*0.0)
        longitudinal_cyclic = 0.0
        # Clamp to reasonable limits (in radians)
        return np.clip([
            collective,              # u0
            lateral_cyclic,          # u1 (longitudinal swashplate)
            longitudinal_cyclic,     # u2 (lateral swashplate)
            tail_rotor               # u3
        ], np.deg2rad([-5, -5, -5, -30]), np.deg2rad([20, 5, 5, 30]))
