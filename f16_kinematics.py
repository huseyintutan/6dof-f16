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
