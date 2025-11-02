# gravity_model.py
# -------------------------------------------------------------
# Gravity models and gravity vector helpers:
# - WGS-84 normal gravity on the ellipsoid (Somigliana formula)
# - Simple height correction
# - Gravity vector in NED and Body frames

import numpy as np
from .earth_model import WGS84, dcm_ned_to_body

# Somigliana constants for WGS-84 normal gravity
_g_e = 9.7803253359       # [m/s^2] gravity at equator
_k   = 0.00193185265241   # Somigliana k
e2   = WGS84["e2"]

def normal_gravity(lat_rad, h_m=0.0):
    """
    WGS-84 normal gravity on the ellipsoid with a simple height correction.
    lat_rad: geodetic latitude [rad]
    h_m: height above ellipsoid [m]
    Returns gravity magnitude [m/s^2].
    """
    s = np.sin(lat_rad)
    # Somigliana on ellipsoid surface
    g0 = _g_e * (1 + _k * s*s) / np.sqrt(1 - e2 * s*s)
    # Free-air correction (first-order): decrease ~3.086e-6 * h
    g = g0 - 3.086e-6 * h_m
    return g

def gravity_ned(lat_rad, h_m=0.0):
    """
    Gravity vector in NED frame. NED convention uses +Down.
    Returns np.array([0, 0, g_down]).
    """
    g = normal_gravity(lat_rad, h_m)
    return np.array([0.0, 0.0, g], dtype=float)

def gravity_body(lat_rad, h_m, phi, theta, psi):
    """
    Gravity vector in Body frame given Euler angles (roll=phi, pitch=theta, yaw=psi).
    Steps: g_ned -> g_body via DCM (NED->Body).
    """
    g_ned = gravity_ned(lat_rad, h_m)
    R_bn = dcm_ned_to_body(phi, theta, psi)  # Body <- NED
    return R_bn @ g_ned
