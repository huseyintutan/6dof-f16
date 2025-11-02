# earth_model.py
# -------------------------------------------------------------
# WGS-84 Earth model utilities:
# - Ellipsoid constants
# - Geodetic <-> ECEF conversions
# - ECEF <-> NED direction cosine matrices (DCMs)
#
# Angles in radians unless otherwise stated.

import numpy as np

# WGS-84 ellipsoid
WGS84 = {
    "a": 6378137.0,                 # [m] semi-major axis
    "f": 1.0 / 298.257223563,       # flattening
    "omega": 7.2921150e-5,          # [rad/s] Earth rotation rate
    "mu": 3.986004418e14,           # [m^3/s^2] GM
}
WGS84["b"]  = WGS84["a"] * (1.0 - WGS84["f"])
WGS84["e2"] = 2*WGS84["f"] - WGS84["f"]**2           # first eccentricity^2

def geodetic_to_ecef(lat, lon, h):
    """
    lat, lon [rad], h [m]  -> x, y, z in ECEF [m]
    """
    a, e2 = WGS84["a"], WGS84["e2"]
    s, c = np.sin(lat), np.cos(lat)
    N = a / np.sqrt(1 - e2 * s*s)          # prime vertical radius
    x = (N + h) * c * np.cos(lon)
    y = (N + h) * c * np.sin(lon)
    z = (N*(1 - e2) + h) * s
    return np.array([x, y, z], dtype=float)

def ecef_to_geodetic(x, y, z):
    """
    Bowring's closed-form approximation (good for all latitudes).
    Returns lat, lon [rad], h [m].
    """
    a, f, b, e2 = WGS84["a"], WGS84["f"], WGS84["b"], WGS84["e2"]
    lon = np.arctan2(y, x)
    r = np.hypot(x, y)
    E2 = a*a - b*b
    F = 54 * b*b * z*z
    G = r*r + (1 - e2) * z*z - e2 * E2
    c = (e2*e2 * F * r*r) / (G*G*G)
    s = (1 + c + np.sqrt(c*c + 2*c))**(1/3)
    P = F / (3 * (s + 1/s + 1)**2 * G*G)
    Q = np.sqrt(1 + 2*e2*e2 * P)
    r0 = -(P*e2*r) / (1 + Q) + np.sqrt(0.5*a*a*(1 + 1/Q) - (P*(1 - e2)*z*z)/(Q*(1 + Q)) - 0.5*P*r*r)
    U = np.sqrt((r - e2*r0)**2 + z*z)
    V = np.sqrt((r - e2*r0)**2 + (1 - e2)*z*z)
    z0 = b*b * z / (a * V)
    h = U * (1 - b*b/(a*V))
    lat = np.arctan2(z + (e2 * z0), r)
    return lat, lon, h

def dcm_ecef_to_ned(lat, lon):
    """
    Direction Cosine Matrix (DCM) from ECEF to NED at given geodetic lat/lon.
    R_en * r_ecef = r_ned
    """
    sL, cL = np.sin(lat), np.cos(lat)
    sB, cB = np.sin(lon), np.cos(lon)
    R = np.array([
        [-sL*cB, -sL*sB,  cL],
        [   -sB,     cB,  0.0],
        [-cL*cB, -cL*sB, -sL]
    ], dtype=float)
    return R

def dcm_ned_to_ecef(lat, lon):
    """Transpose of ECEF->NED."""
    return dcm_ecef_to_ned(lat, lon).T

def dcm_ned_to_body(phi, theta, psi):
    """
    NED -> Body DCM using 3-2-1 (yaw-pitch-roll) Euler sequence.
    Body axes: x-forward, y-right, z-down.
    """
    sφ, cφ = np.sin(phi),   np.cos(phi)
    sθ, cθ = np.sin(theta), np.cos(theta)
    sψ, cψ = np.sin(psi),   np.cos(psi)

    Rz = np.array([[ cψ, sψ, 0],
                   [-sψ, cψ, 0],
                   [  0,  0, 1]])
    Ry = np.array([[ cθ, 0, -sθ],
                   [  0, 1,   0],
                   [ sθ, 0,  cθ]])
    Rx = np.array([[1,   0,   0],
                   [0,  cφ,  sφ],
                   [0, -sφ,  cφ]])
    # NED->Body = Rx * Ry * Rz
    return Rx @ Ry @ Rz
