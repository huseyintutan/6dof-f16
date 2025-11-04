# f16_kinematics.py
# -------------------------------------------------------------
# Kinematics utilities for a 6-DOF aircraft model (NED convention).
#
# Frames & conventions:
#   - Body axes:  x-forward, y-right, z-down
#   - NED frame:  +N (north), +E (east), +D (down)
#   - Euler sequence: yaw(psi) → pitch(theta) → roll(phi)  (3-2-1)
#
# Provided functions:
#   - dcm_body_to_ned(phi, theta, psi): 3x3 DCM, Body → NED
#   - ned_dot_from_body(u, v, w, phi, theta, psi): [Ndot, Edot, Ddot]
#   - integrate_position(N, E, D, u, v, w, phi, theta, psi, dt): next (N,E,D)
#
# Notes:
#   - These are purely kinematic relations (no dynamics).
#   - Use with state derivatives from your dynamics to step the position.
# -------------------------------------------------------------

import math
import numpy as np


def dcm_body_to_ned(phi: float, theta: float, psi: float) -> np.ndarray:
    """
    Direction Cosine Matrix for Body → NED with Euler 3-2-1 (psi-theta-phi).
    Input angles in radians.
    """
    sphi,  cphi  = math.sin(phi),   math.cos(phi)
    sthe,  cthe  = math.sin(theta), math.cos(theta)
    spsi,  cpsi  = math.sin(psi),   math.cos(psi)

    # NED <- Body = R_n^b = (R_z(psi) * R_y(theta) * R_x(phi))^T  = R_x^T * R_y^T * R_z^T
    # Explicit Body→NED DCM (x_fwd, y_right, z_down; NED +Down):
    R = np.array([
        [ cthe*cpsi,                       cthe*spsi,                      -sthe     ],
        [ sphi*sthe*cpsi - cphi*spsi,     sphi*sthe*spsi + cphi*cpsi,      sphi*cthe],
        [ cphi*sthe*cpsi + sphi*spsi,     cphi*sthe*spsi - sphi*cpsi,      cphi*cthe]
    ], dtype=float)
    return R


# ---------------- Quaternion utilities (Body orientation) ----------------
# Quaternion convention: q = [w, x, y, z], rotation from Body to NED computed via DCM(q)

def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n <= 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n

def euler_to_quat(phi: float, theta: float, psi: float) -> np.ndarray:
    c1, s1 = math.cos(psi/2),   math.sin(psi/2)
    c2, s2 = math.cos(theta/2), math.sin(theta/2)
    c3, s3 = math.cos(phi/2),   math.sin(phi/2)
    w = c1*c2*c3 + s1*s2*s3
    x = c1*c2*s3 - s1*s2*c3
    y = c1*s2*c3 + s1*c2*s3
    z = s1*c2*c3 - c1*s2*s3
    return quat_normalize(np.array([w, x, y, z], dtype=float))

def quat_to_euler(q: np.ndarray) -> tuple[float, float, float]:
    q = quat_normalize(q)
    w, x, y, z = q
    # 3-2-1 (yaw, pitch, roll)
    t0 = 2*(w*x + y*z)
    t1 = 1 - 2*(x*x + y*y)
    phi = math.atan2(t0, t1)

    t2 = 2*(w*y - z*x)
    t2 = 1.0 if t2 > 1.0 else (-1.0 if t2 < -1.0 else t2)
    theta = math.asin(t2)

    t3 = 2*(w*z + x*y)
    t4 = 1 - 2*(y*y + z*z)
    psi = math.atan2(t3, t4)
    return phi, theta, psi

def quat_dcm_body_to_ned(q: np.ndarray) -> np.ndarray:
    q = quat_normalize(q)
    w, x, y, z = q
    # DCM (NED <- Body)
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y + w*z),     2*(x*z - w*y)],
        [    2*(x*y - w*z), 1 - 2*(x*x + z*z),     2*(y*z + w*x)],
        [    2*(x*z + w*y),     2*(y*z - w*x), 1 - 2*(x*x + y*y)],
    ], dtype=float)
    return R

def ned_dot_from_body_quat(u: float, v: float, w_b: float, q: np.ndarray) -> tuple[float, float, float]:
    R_nb = quat_dcm_body_to_ned(q)
    V_b  = np.array([u, v, w_b], dtype=float)
    V_n  = R_nb @ V_b
    return float(V_n[0]), float(V_n[1]), float(V_n[2])


def ned_dot_from_body(u: float, v: float, w: float,
                      phi: float, theta: float, psi: float) -> tuple[float, float, float]:
    """
    Convert body-axis velocity [u,v,w] (m/s) to NED velocity components [Ndot, Edot, Ddot] (m/s).
    Angles in radians.
    """
    R_nb = dcm_body_to_ned(phi, theta, psi)  # NED <- Body
    V_b  = np.array([u, v, w], dtype=float)
    V_n  = R_nb @ V_b
    return float(V_n[0]), float(V_n[1]), float(V_n[2])


def integrate_position(N: float, E: float, D: float,
                       u: float, v: float, w: float,
                       phi: float, theta: float, psi: float,
                       dt: float) -> tuple[float, float, float]:
    """
    First-order (Euler) integration of position in a local NED frame.
      (N,E,D)_{k+1} = (N,E,D)_k + (Ndot,Edot,Ddot) * dt
    Returns next (N, E, D).
    """
    Ndot, Edot, Ddot = ned_dot_from_body(u, v, w, phi, theta, psi)
    return N + Ndot*dt, E + Edot*dt, D + Ddot*dt
