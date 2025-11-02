# f16_forces.py
# Aerodynamic forces & moments in BODY axes for the F-16.
# Returns a dict: {"Fx","Fy","Fz","L","M","N"} with +x forward, +y right, +z down.
#
# Dependencies:
#   - f16_constants.F16_CONSTANTS : expects keys "S", "b", "c_bar"
#   - atmosphere (optional): isa_density(h), speed_of_sound(h)
#   - F16AeroDB (passed in as aero_db) with methods:
#       cl(alpha_deg, mach, de_deg), cd(alpha_deg, mach), cm(alpha_deg, mach, de_deg)
#
# Notes:
#   * alpha,beta defined from body velocities (u,v,w)
#   * Body-axes projection uses standard small-aircraft sign conventions (z positive down)
#   * If database calls fail, simple physically-plausible fallbacks are used

import math
from f16_constants import F16_CONSTANTS as C

# ---- Atmosphere helpers (use your atmosphere.py if available) ----
try:
    from atmosphere import isa_density as _rho
    from atmosphere import speed_of_sound as _a
    def rho_at(h_m: float) -> float: return _rho(h_m)
    def a_at(h_m: float)   -> float: return _a(h_m)
except Exception:
    # Simple ISA-like approximations (good enough for low–mid altitude)
    def rho_at(h_m: float) -> float:
        # ρ ≈ 1.225 * exp(-h/8500)  [kg/m^3]
        return 1.225 * math.exp(-max(0.0, h_m) / 8500.0)
    def a_at(h_m: float) -> float:
        # a = sqrt(gamma*R*T), T ≈ 288.15 - 0.0065*h  [K], clamp to 180 K
        T = 288.15 - 0.0065 * max(0.0, h_m)
        T = max(180.0, T)
        return math.sqrt(1.4 * 287.05 * T)

# ---- Basic kinematics helpers ----
def uvw_to_alphabeta(u: float, v: float, w: float):
    """
    Compute angle-of-attack (deg), sideslip (deg), and airspeed magnitude (m/s)
    from body-axis velocities.
    """
    V = max(1e-6, (u*u + v*v + w*w) ** 0.5)
    alpha = math.atan2(w, u)                       # rad
    beta  = math.asin(max(-1.0, min(1.0, v / V)))  # rad
    return math.degrees(alpha), math.degrees(beta), V

# ---- Aerodynamic coefficient lookup (DB-backed with safe fallbacks) ----
def aero_lookup(db, alpha_deg: float, beta_deg: float, controls: dict, h_m: float, V: float):
    """
    Pull CL, CD, Cm from JSON DB via F16AeroDB.
    Yaw/roll coefficients are optional (0 if not provided).
    """
    a = a_at(h_m)
    M = V / max(1e-3, a)
    de = float(controls.get("de", 0.0))

    # --- CL
    try:
        CL = float(db.cl(alpha_deg, M, de))
    except Exception:
        # Reasonable default slope around small angles:
        # CL ≈ 0.2 + 0.08*alpha + 0.02*de
        CL = 0.2 + 0.08 * alpha_deg + 0.02 * de

    # --- CD
    try:
        CD = float(db.cd(alpha_deg, M))
    except Exception:
        # Parabolic drag polar fallback
        CD0, k = 0.02, 0.07
        CD = CD0 + k * (CL ** 2)

    # --- Cm
    try:
        Cm = float(db.cm(alpha_deg, M, de))
    except Exception:
        # Nose-down with positive alpha, elevator nose-down effectiveness
        Cm = -0.02 * alpha_deg - 0.01 * de

    # Lateral-axis (optional): set to zero unless your DB supplies them
    CY = 0.0
    Cl = 0.0
    Cn = 0.0

    return CL, CD, CY, Cl, Cm, Cn

# ---- Main API ----
def aero_forces_moments(u: float, v: float, w: float,
                        p: float, q: float, r: float,
                        h: float,
                        controls: dict,
                        aero_db):
    """
    Compute BODY-axis aerodynamic forces and moments.
    Moments are about the body axes at CM using (S,b,c_bar) scaling.

    Returns:
        dict with keys: Fx, Fy, Fz, L, M, N
    """
    # Geometry
    S  = float(C["S"])
    b  = float(C["b"])
    c_ = float(C["c_bar"])  # NOTE: consistent with constants "c_bar"

    # Angles and dynamic pressure
    alpha_deg, beta_deg, V = uvw_to_alphabeta(u, v, w)
    rho  = rho_at(max(0.0, h))
    qbar = 0.5 * rho * V * V

    # Aero coefficients (from DB + fallbacks)
    CL, CD, CY, Cl, Cm, Cn = aero_lookup(aero_db, alpha_deg, beta_deg, controls, h, V)

    # Body-axis projection (z is positive down):
    # X = -qS * CD * cosα + qS * CL * sinα
    # Z = -qS * CD * sinα - qS * CL * cosα
    # Y =  qS * CY
    alpha = math.radians(alpha_deg)
    cA, sA = math.cos(alpha), math.sin(alpha)

    X = -qbar * S * (CD * cA) + qbar * S * (CL * sA)
    Z = -qbar * S * (CD * sA) - qbar * S * (CL * cA)
    Y =  qbar * S * CY

    # Moments (about body axes)
    L_m = qbar * S * b  * Cl
    M_m = qbar * S * c_ * Cm
    N_m = qbar * S * b  * Cn

    return {"Fx": X, "Fy": Y, "Fz": Z, "L": L_m, "M": M_m, "N": N_m}
