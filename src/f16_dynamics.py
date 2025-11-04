# f16_dynamics.py
# 6-DoF rigid-body dynamics for F-16 (body-axis formulation)
# Provides:
#   - f_dot(state, controls, aero_db, …): state derivatives
#   - trim_level_flight(V_target, h_target, …): simple trim solver
#
# State dict keys:    u,v,w,p,q,r,phi,theta,psi,h
# Control dict keys:  de, da, dr   (degrees)
#
# Requires:
#   f16_constants.F16_CONSTANTS  -> mass, I (or components), g, Tmax_alt, V_trim, h_trim
#   f16_forces.aero_forces_moments(), uvw_to_alphabeta()

import math
import numpy as np
import inspect
from .f16_constants import F16_CONSTANTS as C
from .f16_forces import aero_forces_moments, uvw_to_alphabeta
from .gravity_model import gravity_ned


def _dcm_body_to_ned(phi, theta, psi):
    """Direction cosine matrix from BODY to NED (x-forward, y-right, z-down)."""
    cphi, sphi = math.cos(phi), math.sin(phi)
    cth,  sth  = math.cos(theta), math.sin(theta)
    cpsi, spsi = math.cos(psi), math.sin(psi)

    # 3-2-1 sequence: psi (Z), theta (Y), phi (X)
    Rz = np.array([[ cpsi, spsi, 0.0],
                   [-spsi, cpsi, 0.0],
                   [ 0.0,  0.0,  1.0]])
    Ry = np.array([[ cth, 0.0, -sth],
                   [ 0.0, 1.0,  0.0],
                   [ sth, 0.0,  cth]])
    Rx = np.array([[1.0, 0.0,  0.0],
                   [0.0, cphi, sphi],
                   [0.0,-sphi, cphi]])
    return Rz @ Ry @ Rx  # body -> NED


def _euler_kinematics(p, q, r, phi, theta):
    """Euler angle time derivatives from body rates."""
    sphi, cphi = math.sin(phi), math.cos(phi)
    cth = max(math.cos(theta), 1e-6)  # guard near 90 deg
    tth = math.tan(theta)

    phi_dot   = p + q*sphi*tth + r*cphi*tth
    theta_dot = q*cphi - r*sphi
    psi_dot   = (q*sphi + r*cphi) / cth
    return phi_dot, theta_dot, psi_dot


# ---------- Adapter: call aero_forces_moments with whichever signature user has ----------
def _call_aero_forces_moments(u, v, w, p, q, r, h, controls, aero_db, lef=False, speedbrake=0.0):
    """
    Tries common signatures in order, to avoid TypeError:
      1) (u,v,w,p,q,r,h,controls,aero_db,lef,speedbrake)
      2) (u,v,w,p,q,r,h,controls,aero_db)
      3) (state_dict, controls, aero_db)
      4) (u,v,w,controls,aero_db)
    Returns dict with keys Fx,Fy,Fz,L,M,N (adapts tuple to dict if needed).
    """
    # Try various signatures
    try:
        fm = aero_forces_moments(u, v, w, p, q, r, h, controls, aero_db, lef, speedbrake)
    except TypeError:
        try:
            fm = aero_forces_moments(u, v, w, p, q, r, h, controls, aero_db)
        except TypeError:
            try:
                state = {"u":u, "v":v, "w":w, "p":p, "q":q, "r":r, "h":h}
                fm = aero_forces_moments(state, controls, aero_db)
            except TypeError:
                fm = aero_forces_moments(u, v, w, controls, aero_db)

    # Normalize output to dict
    if isinstance(fm, dict):
        return fm
    else:
        # Expect a 6-tuple/list: (Fx, Fy, Fz, L, M, N)
        try:
            Fx, Fy, Fz, L, M, N = fm
            return {"Fx":Fx, "Fy":Fy, "Fz":Fz, "L":L, "M":M, "N":N}
        except Exception as e:
            raise TypeError(
                "aero_forces_moments returned unsupported type. "
                "Expected dict or 6-tuple (Fx,Fy,Fz,L,M,N)."
            ) from e


def f_dot(state,
          controls,
          aero_db,
          thrust_N=0.0,
          lef=False,
          speedbrake=0.0,
          lat_deg=41.0):
    """
    Compute state derivatives for 6-DoF rigid body.
    Inputs:
        state:    dict(u,v,w,p,q,r,phi,theta,psi,h)
        controls: dict(de, da, dr) [deg]
        aero_db:  F16AeroDB instance (or compatible)
        thrust_N: engine thrust along +x_body (approx)
    Returns:
        dict of time-derivatives (+ h_dot)
    """
    u = float(state["u"]); v = float(state["v"]); w = float(state["w"])
    p = float(state["p"]); q = float(state["q"]); r = float(state["r"])
    phi = float(state["phi"]); theta = float(state["theta"]); psi = float(state["psi"])
    h   = float(state["h"])

    m = float(C["mass"])
    # Gravity magnitude varies with latitude/altitude
    lat_rad = math.radians(float(lat_deg))

    # --- robust inertia fetch: allow either full matrix or individual components
    if "I" in C:
        I = np.array(C["I"], dtype=float)
    else:
        I_xx = float(C["I_xx"]); I_yy = float(C["I_yy"]); I_zz = float(C["I_zz"])
        I_xz = float(C.get("I_xz", 0.0)); I_yx = float(C.get("I_yx", 0.0)); I_yz = float(C.get("I_yz", 0.0))
        I = np.array([
            [ I_xx,   -I_yx,  -I_xz ],
            [ -I_yx,   I_yy,  -I_yz ],
            [ -I_xz,  -I_yz,   I_zz ],
        ], dtype=float)

    # 1) Aerodynamic + Propulsive forces/moments in BODY
    fm = _call_aero_forces_moments(u, v, w, p, q, r, h, controls, aero_db, lef, speedbrake)
    Fx = float(fm.get("Fx", 0.0)) + float(thrust_N)  # thrust on +x_body
    Fy = float(fm.get("Fy", 0.0))
    Fz = float(fm.get("Fz", 0.0))
    L  = float(fm.get("L",  0.0))
    M  = float(fm.get("M",  0.0))
    N  = float(fm.get("N",  0.0))

    # 2) Gravity in BODY (NED gravity is +mg in +z_ned)
    R_bn = _dcm_body_to_ned(phi, theta, psi)  # body->NED
    R_nb = R_bn.T                              # NED->body
    g_ned  = m * gravity_ned(lat_rad, h)  # [N] in NED (down-positive)
    g_body = R_nb @ g_ned
    Fx += g_body[0]; Fy += g_body[1]; Fz += g_body[2]

    # 3) Translational accelerations (including omega x (mV))
    Vb = np.array([u, v, w], dtype=float)
    Fb = np.array([Fx, Fy, Fz], dtype=float)
    omega = np.array([p, q, r], dtype=float)
    Vb_dot = (Fb - np.cross(omega, m * Vb)) / m
    u_dot, v_dot, w_dot = float(Vb_dot[0]), float(Vb_dot[1]), float(Vb_dot[2])

    # 4) Rotational dynamics: I*omega_dot + omega x (I*omega) = M
    Mb = np.array([L, M, N], dtype=float)
    Iomega = I @ omega
    omega_dot = np.linalg.solve(I, (Mb - np.cross(omega, Iomega)))
    p_dot, q_dot, r_dot = float(omega_dot[0]), float(omega_dot[1]), float(omega_dot[2])

    # 5) Euler kinematics
    phi_dot, theta_dot, psi_dot = _euler_kinematics(p, q, r, phi, theta)

    # 6) Altitude rate (h_dot = -w_ned)
    Vn = R_bn @ Vb
    h_dot = -float(Vn[2])

    return {
        "u_dot": u_dot, "v_dot": v_dot, "w_dot": w_dot,
        "p_dot": p_dot, "q_dot": q_dot, "r_dot": r_dot,
        "phi_dot": phi_dot, "theta_dot": theta_dot, "psi_dot": psi_dot,
        "h_dot": h_dot
    }


# -------------------- TRIM SOLVER ----------------------------

def trim_level_flight(V_target, h_target, aero_db,
                      lef=False, speedbrake=0.0, lat_deg=41.0):
    """
    Solve a near-level, near-zero-rate trim at (V_target, h_target).
    Unknowns: alpha (rad), theta (rad), de (deg), thrust_N.
    Residuals ~ 0: u_dot, w_dot, q_dot, h_dot.

    Returns:
        dict: {theta, de, thrust_N, alpha_deg, state0, controls0}
    """
    # Initial guesses
    alpha = math.radians(2.0)
    theta = math.radians(2.0)
    de    = 0.0
    thrust = 0.5 * float(C["Tmax_alt"])

    def residuals(alpha_, theta_, de_, thrust_):
        # Split V into body components from alpha
        u = V_target * math.cos(alpha_)
        w = V_target * math.sin(alpha_)
        v = 0.0
        p = q = r = 0.0

        st = {"u":u, "v":v, "w":w, "p":p, "q":q, "r":r,
              "phi":0.0, "theta":theta_, "psi":0.0, "h":h_target}
        ctr = {"de":de_, "da":0.0, "dr":0.0}

        d = f_dot(st, ctr, aero_db, thrust_, lef=lef,
                  speedbrake=speedbrake, lat_deg=lat_deg)
        return np.array([d["u_dot"], d["w_dot"], d["q_dot"], d["h_dot"]], dtype=float), st, ctr

    # Newton iterations
    for _ in range(40):
        R, _, _ = residuals(alpha, theta, de, thrust)

        eps_al = 1e-4
        eps_th = 1e-4
        eps_de = 1e-3
        eps_T  = 5.0

        R_al, _, _ = residuals(alpha + eps_al, theta, de, thrust)
        R_th, _, _ = residuals(alpha, theta + eps_th, de, thrust)
        R_de, _, _ = residuals(alpha, theta, de + eps_de, thrust)
        R_T,  _, _ = residuals(alpha, theta, de, thrust + eps_T)

        J = np.column_stack([
            (R_al - R)/eps_al,
            (R_th - R)/eps_th,
            (R_de - R)/eps_de,
            (R_T  - R)/eps_T
        ])

        try:
            dx = np.linalg.lstsq(J, -R, rcond=None)[0]
        except np.linalg.LinAlgError:
            break

        alpha += dx[0]
        theta += dx[1]
        de    += dx[2]
        thrust+= dx[3]

        # Bounds
        de = float(np.clip(de, -20.0, 20.0))
        thrust = float(np.clip(thrust, 0.0, float(C["Tmax_alt"])) )

        if np.linalg.norm(R) < 1e-2:
            break

    # Build consistent initial state
    u0 = V_target * math.cos(alpha)
    w0 = V_target * math.sin(alpha)
    v0 = 0.0

    alpha_deg, _, _ = uvw_to_alphabeta(u0, v0, w0)

    state0 = {
        "u":u0, "v":v0, "w":w0,
        "p":0.0, "q":0.0, "r":0.0,
        "phi":0.0, "theta":theta, "psi":0.0,
        "h":h_target
    }
    controls0 = {"de":de, "da":0.0, "dr":0.0}

    return {
        "theta":theta, "de":de, "thrust_N":thrust,
        "alpha_deg":alpha_deg,
        "state0":state0, "controls0":controls0
    }
