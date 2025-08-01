import math
import numpy as np
from scipy.spatial.transform import Rotation as R

def quaternion_derivative(q, omega):
    # q = [x, y, z, w]
    # omega = [p, q, r] = body angular rates
    p, q_, r = omega
    x, y, z, w = q

    dqdt = 0.5 * np.array([
         w * p + y * r - z * q_,
         w * q_ + z * p - x * r,
         w * r + x * q_ - y * p,
        -x * p - y * q_ - z * r
    ])
    return dqdt

def euler6dof_step(state, F, M, m, I, dt, a1, b1, a2, b2):
    # Extract previous states
    pos = state["position"]
    vb = state["body_velocity"]
    vp = state["earth_velocity"]
    ab_prev = state["body_acceleration"]
    omgdot_prev = state["angular_acceleration"]
    ang = state["attitude"]
    q = state["attitude_q"]
    omg = state["angular_rate"]

    c4 = math.cos(ang[0])
    c5 = math.cos(ang[1])
    c6 = math.cos(ang[2])
    s4 = math.sin(ang[0])
    s5 = math.sin(ang[1])
    s6 = math.sin(ang[2])

    gravity_fts2 = 32.181
    MG = np.array([-m * gravity_fts2 * s5, m * gravity_fts2 * s4*c5, m * gravity_fts2 * c5*c4])
    F_with_grav = F + MG

    ab = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # Linear acceleration in body frame
    ab[0] = -(omg[1]*vb[2]-omg[2]*vb[1])+ F_with_grav[0] / m
    ab[1] = (omg[0]*vb[2]-vb[0]*omg[2])+ F_with_grav[1] / m
    ab[2] = (vb[0]*omg[1]-omg[0]*vb[1])+ F_with_grav[2] / m
    ab[3] = M[0] / I[0,0]
    ab[4] = M[1] / I[1,1] - omg[0]*omg[2]*(I[0,0]-I[2,2])/I[1,1]+(omg[2]**2.0-omg[0]**2.0)*I[0,2]/I[1,1]
    ab[5] = M[2] / I[2,2] - I[0,2]*ab[3]/I[2,2]

    for i in range(3):
        vb[i] = vb[i] + dt * (a1 * ab[i] + b1 * ab_prev[i])
        omg[i] = omg[i] + dt * (a1 * ab[i+3] + b1 * omgdot_prev[i])

    ve = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    ve[0] = (vb[0] * c5 + vb[2] *s5) * c4*c6
    ve[1] = vb[1] * c6 + vb[0] *s6
    ve[2] = (vb[0] * s5 - vb[2] *c5) * c4
    ve[3] = omg[0] + (omg[1] * s4 + omg[2] *c4) * math.tan(ang[1])
    ve[4] = omg[1] * c4 - omg[2] *s4
    ve[5] = (omg[2] * c4 + omg[1] *s4)/c5

    xe = np.array([0.0, 0.0, 0.0])

    dqdt = quaternion_derivative(q, omg)
    q_new = q + dqdt * dt
    q_new = q_new / np.linalg.norm(q_new)  # Normalize to prevent drift

    euler_angles = R.from_quat([q_new[0], q_new[1], q_new[2],  q_new[3]]).as_euler("xyz")
    for i in range(3):
        xe[i] = pos[i] + dt * (a2 * ve[i] + b2 * vp[i])

    # Euler integration
    new_state = {
        "position": xe,
        "body_velocity": vb[:3],
        "body_acceleration": ab[:3],
        "earth_velocity": ve[:3],
        "angular_rate": omg,
        "angular_acceleration": ab[3:6],
        "attitude": euler_angles,
        "attitude_q": q_new
    }

    return new_state
