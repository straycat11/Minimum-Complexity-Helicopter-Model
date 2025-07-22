import numpy as np

def skew(v):
    """ Returns the skew-symmetric matrix of a vector v """
    return np.array([
        [ 0,    -v[2],  v[1]],
        [ v[2],  0,    -v[0]],
        [-v[1], v[0],   0 ]
    ])

def rotation_matrix_euler(phi, theta, psi):
    """ Returns the direction cosine matrix from body to inertial frame """
    c, s = np.cos, np.sin

    R = np.array([
        [c(theta)*c(psi), c(theta)*s(psi), -s(theta)],
        [s(phi)*s(theta)*c(psi)-c(phi)*s(psi), s(phi)*s(theta)*s(psi)+c(phi)*c(psi), s(phi)*c(theta)],
        [c(phi)*s(theta)*c(psi)+s(phi)*s(psi), c(phi)*s(theta)*s(psi)-s(phi)*c(psi), c(phi)*c(theta)]
    ])
    return R

def angular_rates_to_euler_dot(phi, theta, omega):
    """ Converts body angular rates to Euler angle rates """
    p, q, r = omega
    t, s, c = np.tan, np.sin, np.cos

    T = np.array([
        [1, s(phi)*t(theta), c(phi)*t(theta)],
        [0, c(phi),         -s(phi)],
        [0, s(phi)/c(theta), c(phi)/c(theta)]
    ])
    return T @ omega

def euler6dof_step(state, F, M, m, I, dt, a1, b1):
    """
    Performs one integration step using Euler's method for rigid body 6-DOF.
    
    Parameters:
        state - dictionary containing:
            position: np.array([x, y, z])
            velocity: np.array([u, v, w]) in body frame
            attitude: np.array([phi, theta, psi]) (Euler angles)
            angular_rate: np.array([p, q, r])
        F - np.array([Fx, Fy, Fz]) total force in body frame
        M - np.array([Mx, My, Mz]) total moment in body frame
        m - mass
        I - 3x3 inertia matrix (assumed diagonal)
        dt - time step (s)
    """
    # Extract previous states
    pos = state["position"]
    vb = state["body_velocity"]
    ap = state["body_acceleration"]
    ang = state["attitude"]
    omg = state["angular_rate"]

    ab = []
    # Linear acceleration in body frame
    ab[0] = -(vb[4]*vb[2]-vb[5]*vb[1])+ F[0] / m
    ab[1] = (vb[3]*vb[2]-vb[0]*vb[5])+ F[1] / m
    ab[2] = (vb[0]*vb[4]-vb[3]*vb[1])+ F[2] / m
    ab[3] = F[3] / I[0,0]
    ab[4] = F[4] / I[1,1] - vb[3]*vb[5]*(I[0,0]-I[2,2])/I[1,1]+(vb[5]**2.0-vb[3]**2.0)*I[0,2]/I[1,1]
    ab[5] = F[5] / I[2,2] - I[0,2]*ab[3]/I[2,2]

    for i in range(5):
        vb[i] = vb[i] + dt * (a1 * ab[i] + b1 * ap[i])

    ve = []
    ve[0] = vb[0] 
    # Angular acceleration
    I_inv = np.linalg.inv(I)
    alpha = I_inv @ (M - np.cross(omg, I @ omg))

    # Position derivative in inertial frame
    R = rotation_matrix_euler(*ang)
    pos_dot = R @ vel

    # Attitude rate from body angular rates
    ang_dot = angular_rates_to_euler_dot(*ang, omg)

    # Euler integration
    new_state = {
        "position": pos + pos_dot * dt,
        "velocity": vel + acc * dt,
        "attitude": ang + ang_dot * dt,
        "angular_rate": omg + alpha * dt
    }

    return new_state
