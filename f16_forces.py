# f16_forces.py
# -------------------------------------------------------------
# From state + controls, compute aerodynamic coefficients (via DB),
# then convert to body-axis forces/moments: X,Y,Z,L,M,N.

import math
from f16_constants import F16_CONSTANTS as C
from f16_atmosphere import get_atmosphere

def uvw_to_alphabeta(u, v, w):
    """
    Body-axis velocities [m/s] -> angles [deg]
      alpha = atan2(w, u)
      beta  = asin(v / V)
    """
    V = math.sqrt(u*u + v*v + w*w) + 1e-9
    alpha = math.degrees(math.atan2(w, max(1e-9, u)))
    beta  = math.degrees(math.asin(max(-1.0, min(1.0, v / V))))
    return alpha, beta, V

def aero_forces_moments(state, controls, aero_db, lef=False, speedbrake=0.0):
    """
    Inputs:
      state: dict with { 'u','v','w','h' [, 'dht'] }  (SI units, meters/radians/seconds)
      controls: dict with {'de','da','dr'} in degrees
      aero_db: instance of F16AeroDB
      lef: bool, include LEF increments
      speedbrake: 0..1 scale
    Returns:
      dict with {'X','Y','Z','L','M','N','q','coeffs'}
    """
    u = state["u"]; v = state["v"]; w = state["w"]
    h = state.get("h", 0.0)
    dht_deg = state.get("dht", 0.0)

    # Atmosphere & dynamic pressure
    atm = get_atmosphere(h)
    rho = atm["rho"]
    alpha_deg, beta_deg, V = uvw_to_alphabeta(u, v, w)
    q_dyn = 0.5 * rho * V * V

    # Aerodynamic coefficients (from DB)
    coeffs = aero_db.coeffs(
        alpha_deg=alpha_deg,
        beta_deg=beta_deg,
        dht_deg=dht_deg,
        controls=controls,
        lef=lef,
        speedbrake=speedbrake
    )

    # Forces (body axes), Moments about body origin
    S, b, cbar = C["S"], C["b"], C["c_bar"]
    X = q_dyn * S * coeffs["CX"]
    Y = q_dyn * S * coeffs["CY"]
    Z = q_dyn * S * coeffs["CZ"]
    L = q_dyn * S * b    * coeffs["Cl"]
    M = q_dyn * S * cbar * coeffs["Cm"]
    N = q_dyn * S * b    * coeffs["Cn"]

    return {"X": X, "Y": Y, "Z": Z, "L": L, "M": M, "N": N, "q": q_dyn, "coeffs": coeffs}
