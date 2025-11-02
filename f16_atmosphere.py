# f16_atmosphere.py
# -------------------------------------------------------------
# Simple International Standard Atmosphere (ISA) model.
# Focus: lower atmosphere with a linear lapse rate (0–11 km).
# For h > 11 km, a minimal isothermal layer is included for continuity.
#
# Usage:
#   from f16_atmosphere import get_atmosphere
#   atm = get_atmosphere(3048.0)   # altitude in meters
#   rho, T, p, a = atm["rho"], atm["T"], atm["p"], atm["a"]
#
# If you keep all files in the same folder, this module will try to
# import constants from f16_constants.py. If not found, it falls back
# to internal ISA defaults.

from math import sqrt, exp

# Try to import shared constants if available (same folder).
try:
    from f16_constants import F16_CONSTANTS as _C
    _HAS_EXTERNAL_CONSTANTS = True
except Exception:
    _C = {}
    _HAS_EXTERNAL_CONSTANTS = False

# ISA defaults (used if f16_constants.py is not present)
_T0   = _C.get("T0",   288.15)     # [K] Sea-level temperature
_p0   = _C.get("p0",   101325.0)   # [Pa] Sea-level pressure
_rho0 = _C.get("rho0", 1.225)      # [kg/m^3] Sea-level density (informational)
_g0   = _C.get("g",    9.80665)    # [m/s^2] Gravity
_R    = _C.get("R",    287.05287)  # [J/(kg·K)] Gas constant for air
_gamma= _C.get("gamma",1.4)        # [-] Specific heat ratio for air

# Troposphere lapse rate (0–11 km): T = T0 + aL*h
# Note: aL is negative in the troposphere.
_aL   = -0.0065                     # [K/m]

# Tropopause top for the simple piecewise model:
_H_TROP = 11000.0                  # [m] 11 km


def _troposphere(h_m: float):
    """
    ISA troposphere (0–11 km) with linear lapse rate.
    Returns (T, p, rho, a).
    """
    # Temperature
    T = _T0 + _aL * h_m

    # Pressure (power law using lapse rate)
    # p = p0 * (T/T0)^(-g0/(aL*R))
    exponent = -_g0 / (_aL * _R)
    p = _p0 * (T / _T0) ** (exponent)

    # Density from ideal gas: rho = p / (R*T)
    rho = p / (_R * T)

    # Speed of sound: a = sqrt(gamma * R * T)
    a = sqrt(_gamma * _R * T)

    return T, p, rho, a


def _isothermal_above_tropopause(h_m: float, T_base: float, p_base: float, h_base: float):
    """
    Minimal isothermal continuation above 11 km (not a full ISA).
    Keeps T constant at T_base and integrates hydrostatic equation:
      p(h) = p_base * exp( -g0*(h - h_base) / (R*T_base) )
    Returns (T, p, rho, a).
    """
    T = T_base
    p = p_base * exp( -_g0 * (h_m - h_base) / (_R * T_base) )
    rho = p / (_R * T)
    a = sqrt(_gamma * _R * T)
    return T, p, rho, a


def get_atmosphere(h_m: float) -> dict:
    """
    Compute ISA state at altitude h_m [m].
    Returns a dict with:
      {
        "rho": density [kg/m^3],
        "T":   temperature [K],
        "p":   pressure [Pa],
        "a":   speed of sound [m/s]
      }

    Model:
      - 0–11 km: linear lapse rate (troposphere).
      - >11 km : simple isothermal continuation for continuity
                 (NOT a full multi-layer ISA).
    """
    h = max(0.0, float(h_m))  # clamp below 0 to sea level

    if h <= _H_TROP:
        T, p, rho, a = _troposphere(h)
    else:
        # Evaluate base at tropopause first, then continue isothermal.
        T11, p11, rho11, a11 = _troposphere(_H_TROP)
        T, p, rho, a = _isothermal_above_tropopause(h, T11, p11, _H_TROP)

    return {"rho": rho, "T": T, "p": p, "a": a}
