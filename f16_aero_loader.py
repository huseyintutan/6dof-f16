# f16_aero_loader.py  (drop-in replacement)
# -------------------------------------------------------------
# F-16 Aerodynamic Database Loader and Interpolator
# Now supports TWO JSON schemas:
#   (A) Legacy/complex:  db["f16_AerodynamicData"]["BP"/"data"...]  (your MAT export)
#   (B) Simple schema:   {"Cx":{...},"Cy":{...},...,"delta_elevator":{...},...}
#
# Usage:
#   db = F16AeroDB("F16_database.json")           # file path
#   # or
#   db = F16AeroDB(already_loaded_dict)           # in-memory dict
#
# API:
#   coeffs(alpha_deg, beta_deg, dht_deg=0, controls=None, lef=False, speedbrake=0.0)
#   -> {'CX','CY','CZ','Cl','Cm','Cn'}
# -------------------------------------------------------------

import json
import numpy as np


# ======================= Interp helpers ======================

def _clamp_idx(x, g):
    if x <= g[0]:
        return 0, 0, 0.0
    if x >= g[-1]:
        return len(g)-1, len(g)-1, 0.0
    lo, hi = 0, len(g)-1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if x >= g[mid]:
            lo = mid
        else:
            hi = mid
    i, j = lo, lo+1
    t = (x - g[i]) / (g[j] - g[i])
    return i, j, float(t)

def _lerp(a, b, t):
    return a + (b - a) * t

def _interp1(x, grid, vals):
    i, j, t = _clamp_idx(x, grid)
    if i == j:
        return float(vals[i])
    return float(_lerp(vals[i], vals[j], t))

def _interp2(x, y, gx, gy, table):
    arr = np.array(table, dtype=float)
    ix0, ix1, tx = _clamp_idx(x, gx)
    iy0, iy1, ty = _clamp_idx(y, gy)
    if arr.shape == (len(gx), len(gy)):
        v00 = arr[ix0, iy0]; v01 = arr[ix0, iy1]
        v10 = arr[ix1, iy0]; v11 = arr[ix1, iy1]
    elif arr.shape == (len(gy), len(gx)):
        v00 = arr[iy0, ix0]; v01 = arr[iy1, ix0]
        v10 = arr[iy0, ix1]; v11 = arr[iy1, ix1]
    else:
        raise ValueError(f"interp2: bad shape {arr.shape}")
    v0 = _lerp(v00, v01, ty)
    v1 = _lerp(v10, v11, ty)
    return float(_lerp(v0, v1, tx))

def _interp3(x, y, z, gx, gy, gz, table):
    arr = np.array(table, dtype=float)
    ix0, ix1, tx = _clamp_idx(x, gx)
    iy0, iy1, ty = _clamp_idx(y, gy)
    iz0, iz1, tz = _clamp_idx(z, gz)
    shapes = (len(gx), len(gy), len(gz))
    perms = [(0,1,2),(0,2,1),(1,0,2),(1,2,0),(2,0,1),(2,1,0)]
    matched = None
    for p in perms:
        if arr.shape == tuple(shapes[i] for i in p):
            matched = p
            break
    if matched is None:
        raise ValueError(f"interp3: bad shape {arr.shape} vs {shapes}")
    def take(ix,iy,iz):
        return float(arr[[ix,iy,iz][matched.index(0)],
                         [ix,iy,iz][matched.index(1)],
                         [ix,iy,iz][matched.index(2)]])
    v000 = take(ix0,iy0,iz0); v001 = take(ix0,iy0,iz1)
    v010 = take(ix0,iy1,iz0); v011 = take(ix0,iy1,iz1)
    v100 = take(ix1,iy0,iz0); v101 = take(ix1,iy0,iz1)
    v110 = take(ix1,iy1,iz0); v111 = take(ix1,iy1,iz1)
    v00 = _lerp(v000, v001, tz); v01 = _lerp(v010, v011, tz)
    v10 = _lerp(v100, v101, tz); v11 = _lerp(v110, v111, tz)
    v0 = _lerp(v00, v01, ty);   v1 = _lerp(v10, v11, ty)
    return float(_lerp(v0, v1, tx))


# ========================= Main class ========================

class F16AeroDB:
    """
    Unified loader for two schemas:
      (A) Legacy complex: db['f16_AerodynamicData']['BP'/'data'...]
      (B) Simple flat:    {'Cx':{'alpha':...,'values':...}, ...}
    """

    def __init__(self, source):
        # source can be a filepath (str) or a dict already loaded
        if isinstance(source, str):
            with open(source, "r") as f:
                db = json.load(f)
        elif isinstance(source, dict):
            db = source
        else:
            raise TypeError("F16AeroDB: source must be filepath (str) or dict")

        # Detect schema
        if "f16_AerodynamicData" in db:
            self._init_legacy(db["f16_AerodynamicData"])
            self.schema = "legacy"
        else:
            self._init_simple(db)
            self.schema = "simple"

    # ---------------- Legacy schema -----------------
    def _init_legacy(self, aero):
        self.bp   = aero["BP"]
        self.data = aero["data"]

        coeffs = ["CX","CY","CZ","Cl","Cm","Cn"]
        self.grids = {}
        self.tabs  = {}
        for k in coeffs:
            self.grids[k] = dict(
                alpha = self.bp[k][k].get("alpha"),
                beta  = self.bp[k][k].get("beta"),
                dHT   = self.bp[k][k].get("dHT"),
            )
            self.tabs[k] = self.data[k][k]

        # Optional control/LEF/speedbrake
        self.deltaCm   = self.data["Cm"].get("deltaCm")
        self.Clda20    = self.data["Cl"].get("Clda20")
        self.Cndr30    = self.data["Cn"].get("Cndr30")
        self.CYda20    = self.data["CY"].get("CYda20")
        self.CXlef     = self.data["CX"].get("CXlef")
        self.CYlef     = self.data["CY"].get("CYlef")
        self.CZlef     = self.data["CZ"].get("CZlef")
        self.deltaCXSB = self.data["CX"].get("deltaCXSB")

    # ---------------- Simple schema -----------------
    def _init_simple(self, db):
        """
        Expected minimal structure, e.g.:
          {
            "Cx": {"alpha":[...], "values":[...]},
            "Cz": {"alpha":[...], "values":[...]},
            "Cm": {"alpha":[...], "values":[...]},
            "Cl": {"beta":[...],  "values":[...]},
            "Cy": {"beta":[...],  "values":[...]},
            "Cn": {"beta":[...],  "values":[...]},
            "delta_elevator": {"de":[...], "Cm_delta_e":[...], "Cz_delta_e":[...]},
            "delta_aileron":  {"da":[...], "Cl_delta_a":[...], "Cn_delta_a":[...]},
            "delta_rudder":   {"dr":[...], "Cy_delta_r":[...], "Cn_delta_r":[...]}
          }
        """
        self.simple = db

        # Build grids/tables to unify API
        self.grids = {
            "CX": {"alpha": db.get("Cx",{}).get("alpha"), "beta": None, "dHT": None},
            "CZ": {"alpha": db.get("Cz",{}).get("alpha"), "beta": None, "dHT": None},
            "Cm": {"alpha": db.get("Cm",{}).get("alpha"), "beta": None, "dHT": None},
            "Cl": {"alpha": None, "beta": db.get("Cl",{}).get("beta"), "dHT": None},
            "CY": {"alpha": None, "beta": db.get("Cy",{}).get("beta"), "dHT": None},
            "Cn": {"alpha": None, "beta": db.get("Cn",{}).get("beta"), "dHT": None},
        }
        self.tabs = {
            "CX": db.get("Cx",{}).get("values"),
            "CZ": db.get("Cz",{}).get("values"),
            "Cm": db.get("Cm",{}).get("values"),
            "Cl": db.get("Cl",{}).get("values"),
            "CY": db.get("Cy",{}).get("values"),
            "Cn": db.get("Cn",{}).get("values"),
        }

        de = db.get("delta_elevator", {})
        da = db.get("delta_aileron",  {})
        dr = db.get("delta_rudder",   {})

        self.de_grid = de.get("de")
        self.Cm_de   = de.get("Cm_delta_e")
        self.Cz_de   = de.get("Cz_delta_e")

        self.da_grid = da.get("da")
        self.Cl_da   = da.get("Cl_delta_a")
        self.Cn_da   = da.get("Cn_delta_a")

        self.dr_grid = dr.get("dr")
        self.Cy_dr   = dr.get("Cy_delta_r")
        self.Cn_dr   = dr.get("Cn_delta_r")

        # Not available in simple schema
        self.CXlef = self.CYlef = self.CZlef = None
        self.deltaCXSB = None
        self.deltaCm = None    # (we use explicit Cm_delta_e instead)

    # ---------------- Common helpers ----------------
    def _interp_coeff(self, coeff: str, a: float, b: float, h: float) -> float:
        tab = self.tabs[coeff]
        g   = self.grids[coeff]
        alpha, beta, dht = g["alpha"], g["beta"], g["dHT"]

        if tab is None:
            # coefficient not present in simple schema → return 0
            return 0.0

        ndim = np.ndim(tab)
        if ndim == 3 and alpha and beta and dht:
            return _interp3(a, b, h, alpha, beta, dht, tab)
        elif ndim == 2 and alpha and beta:
            return _interp2(a, b, alpha, beta, tab)
        elif ndim == 1 and alpha:
            return _interp1(a, alpha, tab)
        elif ndim == 1 and beta:
            return _interp1(b, beta, tab)
        else:
            # simple 1D array without grid? treat as constant (first elem)
            if ndim == 1:
                return float(tab[0])
            raise ValueError(f"Grid mismatch for {coeff}: ndim={ndim}, grids={g}")

    # ---------------- Public API --------------------
    def coeffs(
        self,
        alpha_deg: float,
        beta_deg: float,
        dht_deg: float = 0.0,
        controls: dict | None = None,
        lef: bool = False,
        speedbrake: float = 0.0
    ) -> dict:
        a, b, h = float(alpha_deg), float(beta_deg), float(dht_deg)

        # Base coefficients
        cx = self._interp_coeff("CX", a, b, h)
        cy = self._interp_coeff("CY", a, b, h)
        cz = self._interp_coeff("CZ", a, b, h)
        cl = self._interp_coeff("Cl", a, b, h)
        cm = self._interp_coeff("Cm", a, b, h)
        cn = self._interp_coeff("Cn", a, b, h)

        # Controls
        de = da = dr = 0.0
        if controls:
            de = float(controls.get("de", 0.0))
            da = float(controls.get("da", 0.0))
            dr = float(controls.get("dr", 0.0))

        if self.schema == "legacy":
            # Legacy: use derivative tables if present
            if self.deltaCm is not None and abs(de) > 0:
                gA = self.grids["Cm"]["alpha"]
                cm += _interp1(a, gA, self.deltaCm) * de
            if self.Clda20 is not None and abs(da) > 0:
                scl = da / 20.0
                gA, gB = self.grids["Cl"]["alpha"], self.grids["Cl"]["beta"]
                cl += _interp2(a, b, gA, gB, self.Clda20) * scl
                if self.CYda20 is not None:
                    gA2, gB2 = self.grids["CY"]["alpha"], self.grids["CY"]["beta"]
                    cy += _interp2(a, b, gA2, gB2, self.CYda20) * scl
            if self.Cndr30 is not None and abs(dr) > 0:
                scl = dr / 30.0
                gA3, gB3 = self.grids["Cn"]["alpha"], self.grids["Cn"]["beta"]
                cn += _interp2(a, b, gA3, gB3, self.Cndr30) * scl
            if lef:
                if self.CXlef is not None:
                    cx += _interp2(a, b, self.grids["CX"]["alpha"], self.grids["CX"]["beta"], self.CXlef)
                if self.CYlef is not None:
                    cy += _interp2(a, b, self.grids["CY"]["alpha"], self.grids["CY"]["beta"], self.CYlef)
                if self.CZlef is not None:
                    cz += _interp2(a, b, self.grids["CZ"]["alpha"], self.grids["CZ"]["beta"], self.CZlef)
            if self.deltaCXSB is not None and speedbrake > 0.0:
                cx += _interp1(a, self.grids["CX"]["alpha"], self.deltaCXSB) * float(speedbrake)

        else:
            # Simple schema: apply basic control increments (independent of alpha/beta)
            if self.de_grid is not None and abs(de) > 0:
                if self.Cm_de is not None:
                    cm += _interp1(de, self.de_grid, self.Cm_de)
                if self.Cz_de is not None:
                    cz += _interp1(de, self.de_grid, self.Cz_de)
            if self.da_grid is not None and abs(da) > 0:
                if self.Cl_da is not None:
                    cl += _interp1(da, self.da_grid, self.Cl_da)
                if self.Cn_da is not None:
                    cn += _interp1(da, self.da_grid, self.Cn_da)
            if self.dr_grid is not None and abs(dr) > 0:
                if self.Cy_dr is not None:
                    cy += _interp1(dr, self.dr_grid, self.Cy_dr)
                if self.Cn_dr is not None:
                    cn += _interp1(dr, self.dr_grid, self.Cn_dr)
            # LEF / speedbrake not defined in simple schema → ignored

        return {"CX": cx, "CY": cy, "CZ": cz, "Cl": cl, "Cm": cm, "Cn": cn}
