# f16_aero_loader.py
# -------------------------------------------------------------
# F-16 Aerodynamic Database Loader and Interpolator
# -------------------------------------------------------------
# This module reads aerodynamic lookup tables from the JSON database
# (F16_database.json) and interpolates aerodynamic coefficients:
# CX, CY, CZ, Cl, Cm, and Cn.
#
# Each coefficient may depend on different grid dimensions:
#   - 1D: function of alpha
#   - 2D: function of (alpha, beta)
#   - 3D: function of (alpha, beta, dHT)
#
# Optional control surface and configuration effects are also included:
#   - deltaCm(alpha): stabilator (elevator)
#   - Clda20(alpha,beta): aileron
#   - Cndr30(alpha,beta): rudder
#   - CYda20(alpha,beta): lateral coupling due to aileron
#   - CXlef, CYlef, CZlef: leading-edge flap increments
#   - deltaCXSB(alpha): speedbrake drag increment
#
# Usage Example:
#   from f16_aero_loader import F16AeroDB
#   db = F16AeroDB("F16_database.json")
#   coeffs = db.coeffs(alpha_deg=5, beta_deg=0, dht_deg=0,
#                      controls={"de":0, "da":0, "dr":0},
#                      lef=False, speedbrake=0.0)
#   print(coeffs)
#
# Output:
#   {'CX': -0.0066, 'CY': 0.0, 'CZ': -0.367, 'Cl': 0.0, 'Cm': -0.0498, 'Cn': 0.0}
# -------------------------------------------------------------

import json
import numpy as np


# =============================================================
# --- Helper Functions for Interpolation ---
# =============================================================

def _clamp_idx(x, g):
    """Find the two grid indices [i, i+1] surrounding x and the interpolation weight t."""
    if x <= g[0]:
        return 0, 0, 0.0
    if x >= g[-1]:
        return len(g) - 1, len(g) - 1, 0.0
    lo, hi = 0, len(g) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if x >= g[mid]:
            lo = mid
        else:
            hi = mid
    i, j = lo, lo + 1
    t = (x - g[i]) / (g[j] - g[i])
    return i, j, float(t)


def _lerp(a, b, t):
    """Linear interpolation between a and b."""
    return a + (b - a) * t


def _interp1(x, g, v):
    """1D linear interpolation."""
    i, j, t = _clamp_idx(x, g)
    if i == j:
        return float(v[i])
    return float(_lerp(v[i], v[j], t))


def _interp2(x, y, gx, gy, table):
    """2D bilinear interpolation for table defined on grids gx, gy."""
    arr = np.array(table, dtype=float)
    ix0, ix1, tx = _clamp_idx(x, gx)
    iy0, iy1, ty = _clamp_idx(y, gy)

    # handle possible axis order mismatch
    if arr.shape == (len(gx), len(gy)):
        v00 = arr[ix0, iy0]; v01 = arr[ix0, iy1]
        v10 = arr[ix1, iy0]; v11 = arr[ix1, iy1]
    elif arr.shape == (len(gy), len(gx)):
        v00 = arr[iy0, ix0]; v01 = arr[iy1, ix0]
        v10 = arr[iy0, ix1]; v11 = arr[iy1, ix1]
    else:
        raise ValueError(f"interp2: array shape {arr.shape}, expected ({len(gx)},{len(gy)}) or transpose")

    v0 = _lerp(v00, v01, ty)
    v1 = _lerp(v10, v11, ty)
    return float(_lerp(v0, v1, tx))


def _interp3(x, y, z, gx, gy, gz, table):
    """3D trilinear interpolation for (x,y,z) on grids gx, gy, gz."""
    arr = np.array(table, dtype=float)
    ix0, ix1, tx = _clamp_idx(x, gx)
    iy0, iy1, ty = _clamp_idx(y, gy)
    iz0, iz1, tz = _clamp_idx(z, gz)

    # detect possible axis permutation (alpha,beta,dHT or variants)
    shapes = (len(gx), len(gy), len(gz))
    perms = [(0, 1, 2), (0, 2, 1), (1, 0, 2), (1, 2, 0), (2, 0, 1), (2, 1, 0)]
    matched = None
    for p in perms:
        if arr.shape == tuple(shapes[i] for i in p):
            matched = p
            break
    if matched is None:
        raise ValueError(f"interp3: array shape {arr.shape} not compatible with {shapes}")

    def take(ix, iy, iz):
        return float(arr[
            [ix, iy, iz][matched.index(0)],
            [ix, iy, iz][matched.index(1)],
            [ix, iy, iz][matched.index(2)]
        ])

    v000 = take(ix0, iy0, iz0); v001 = take(ix0, iy0, iz1)
    v010 = take(ix0, iy1, iz0); v011 = take(ix0, iy1, iz1)
    v100 = take(ix1, iy0, iz0); v101 = take(ix1, iy0, iz1)
    v110 = take(ix1, iy1, iz0); v111 = take(ix1, iy1, iz1)

    v00 = _lerp(v000, v001, tz)
    v01 = _lerp(v010, v011, tz)
    v10 = _lerp(v100, v101, tz)
    v11 = _lerp(v110, v111, tz)

    v0 = _lerp(v00, v01, ty)
    v1 = _lerp(v10, v11, ty)
    return float(_lerp(v0, v1, tx))


# =============================================================
# --- Main Class: F16AeroDB ---
# =============================================================

class F16AeroDB:
    """
    Loads the F-16 aerodynamic database (JSON) and returns
    aerodynamic coefficients for given (alpha, beta, dHT)
    and control surface inputs.
    """

    def __init__(self, json_path: str):
        with open(json_path, "r") as f:
            db = json.load(f)

        aero = db["f16_AerodynamicData"]
        self.bp   = aero["BP"]    # breakpoints (grids)
        self.data = aero["data"]  # coefficient data tables

        coeffs = ["CX", "CY", "CZ", "Cl", "Cm", "Cn"]
        self.grids = {}
        self.tabs  = {}

        # Load grid and data for each coefficient
        for k in coeffs:
            self.grids[k] = dict(
                alpha = self.bp[k][k].get("alpha"),
                beta  = self.bp[k][k].get("beta"),
                dHT   = self.bp[k][k].get("dHT"),
            )
            self.tabs[k] = self.data[k][k]

        # Optional derivative tables (control effects)
        self.deltaCm   = self.data["Cm"].get("deltaCm")   # elevator
        self.Clda20    = self.data["Cl"].get("Clda20")    # aileron
        self.Cndr30    = self.data["Cn"].get("Cndr30")    # rudder
        self.CYda20    = self.data["CY"].get("CYda20")    # aileron side-force
        self.CXlef     = self.data["CX"].get("CXlef")     # LEF increments
        self.CYlef     = self.data["CY"].get("CYlef")
        self.CZlef     = self.data["CZ"].get("CZlef")
        self.deltaCXSB = self.data["CX"].get("deltaCXSB") # speedbrake

    # ---------------------------------------------------------
    def _interp_coeff(self, coeff: str, a: float, b: float, h: float) -> float:
        """Interpolate a single coefficient (CX, CY, etc.) depending on table dimension."""
        tab = self.tabs[coeff]
        g   = self.grids[coeff]
        alpha, beta, dht = g["alpha"], g["beta"], g["dHT"]
        ndim = np.ndim(tab)

        if ndim == 3 and alpha and beta and dht:
            return _interp3(a, b, h, alpha, beta, dht, tab)
        elif ndim == 2 and alpha and beta:
            return _interp2(a, b, alpha, beta, tab)
        elif ndim == 1 and alpha:
            return _interp1(a, alpha, tab)
        else:
            raise ValueError(f"Grid mismatch for {coeff}: ndim={ndim}, grids={g}")

    # ---------------------------------------------------------
    def coeffs(
        self,
        alpha_deg: float,
        beta_deg: float,
        dht_deg: float = 0.0,
        controls: dict | None = None,
        lef: bool = False,
        speedbrake: float = 0.0
    ) -> dict:
        """
        Compute aerodynamic coefficients for given inputs.
        Inputs (degrees):
          alpha_deg: angle of attack
          beta_deg: sideslip angle
          dht_deg: horizontal tail deflection
          controls: dict with {'de','da','dr'} for elevator, aileron, rudder
          lef: include leading-edge flap increments if True
          speedbrake: 0..1 scale for speedbrake effect
        Returns:
          dict with {'CX','CY','CZ','Cl','Cm','Cn'}
        """
        a, b, h = float(alpha_deg), float(beta_deg), float(dht_deg)

        # Base coefficients
        cx = self._interp_coeff("CX", a, b, h)
        cy = self._interp_coeff("CY", a, b, h)
        cz = self._interp_coeff("CZ", a, b, h)
        cl = self._interp_coeff("Cl", a, b, h)
        cm = self._interp_coeff("Cm", a, b, h)
        cn = self._interp_coeff("Cn", a, b, h)

        # Control surface inputs
        de = da = dr = 0.0
        if controls:
            de = float(controls.get("de", 0.0))
            da = float(controls.get("da", 0.0))
            dr = float(controls.get("dr", 0.0))

        # Elevator (stabilator) effect
        if self.deltaCm is not None and abs(de) > 0:
            gA = self.grids["Cm"]["alpha"]
            cm += _interp1(a, gA, self.deltaCm) * de

        # Aileron effects (roll + side-force)
        if self.Clda20 is not None and abs(da) > 0:
            scl = da / 20.0
            gA, gB = self.grids["Cl"]["alpha"], self.grids["Cl"]["beta"]
            cl += _interp2(a, b, gA, gB, self.Clda20) * scl
            if self.CYda20 is not None:
                gA2, gB2 = self.grids["CY"]["alpha"], self.grids["CY"]["beta"]
                cy += _interp2(a, b, gA2, gB2, self.CYda20) * scl

        # Rudder effect
        if self.Cndr30 is not None and abs(dr) > 0:
            scl = dr / 30.0
            gA3, gB3 = self.grids["Cn"]["alpha"], self.grids["Cn"]["beta"]
            cn += _interp2(a, b, gA3, gB3, self.Cndr30) * scl

        # LEF increments
        if lef:
            if self.CXlef is not None:
                cx += _interp2(a, b, self.grids["CX"]["alpha"], self.grids["CX"]["beta"], self.CXlef)
            if self.CYlef is not None:
                cy += _interp2(a, b, self.grids["CY"]["alpha"], self.grids["CY"]["beta"], self.CYlef)
            if self.CZlef is not None:
                cz += _interp2(a, b, self.grids["CZ"]["alpha"], self.grids["CZ"]["beta"], self.CZlef)

        # Speedbrake drag increment
        if self.deltaCXSB is not None and speedbrake > 0.0:
            cx += _interp1(a, self.grids["CX"]["alpha"], self.deltaCXSB) * float(speedbrake)

        return {
            "CX": cx,
            "CY": cy,
            "CZ": cz,
            "Cl": cl,
            "Cm": cm,
            "Cn": cn
        }
