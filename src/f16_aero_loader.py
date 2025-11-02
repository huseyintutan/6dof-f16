# f16_aero_loader.py
# JSON → CL/CD/Cm interpolasyon (otomatık grid bulma + nested anahtar arama)
import json
import bisect
from collections.abc import Mapping, Sequence

# ---------- tiny utils ----------
def _is_array_like(x):
    return isinstance(x, (list, tuple)) and (len(x) == 0 or not isinstance(x[0], Mapping))

def _flatten_keys(d, prefix=""):
    """Debug amaçlı: JSON içindeki tüm yolları döndürür."""
    out = []
    if isinstance(d, Mapping):
        for k, v in d.items():
            p = f"{prefix}.{k}" if prefix else k
            out.append(p)
            out.extend(_flatten_keys(v, p))
    elif isinstance(d, Sequence) and not isinstance(d, (str, bytes)):
        # çok büyük array’leri açmayalım; dizinin “türü” için bir not düş
        out.append(prefix + "[]")
    return out

def _deep_find_any(d, keys):
    """Sözlük içinde (nested) anahtarlardan herhangi birini bul ve değeri döndür."""
    if not isinstance(d, Mapping):
        return None
    # 1) düz seviyede dene
    for k in keys:
        if k in d:
            return d[k]
    # 2) nesting
    for v in d.values():
        if isinstance(v, Mapping):
            got = _deep_find_any(v, keys)
            if got is not None:
                return got
    return None

# ---------- interpolation ----------
def _interp1d(xg, yg, x):
    if len(xg) != len(yg):
        raise ValueError("interp1d: grid/value size mismatch")
    if x <= xg[0]:  return yg[0]
    if x >= xg[-1]: return yg[-1]
    i = bisect.bisect_left(xg, x) - 1
    x0, x1 = xg[i], xg[i+1]
    y0, y1 = yg[i], yg[i+1]
    t = 0.0 if x1 == x0 else (x - x0) / (x1 - x0)
    return y0 + t*(y1 - y0)

def _clamp_idx(grid, val):
    if val <= grid[0]:  return 0
    if val >= grid[-1]: return len(grid)-2
    return bisect.bisect_left(grid, val) - 1

def _interp2d(xg, yg, Z, x, y):
    ix0 = _clamp_idx(xg, x)
    iy0 = _clamp_idx(yg, y)
    x0, x1 = xg[ix0], xg[ix0+1]
    y0, y1 = yg[iy0], yg[iy0+1]
    tx = 0.0 if x1==x0 else (x - x0)/(x1 - x0)
    ty = 0.0 if y1==y0 else (y - y0)/(y1 - y0)

    z00 = Z[ix0][iy0]
    z01 = Z[ix0][iy0+1]
    z10 = Z[ix0+1][iy0]
    z11 = Z[ix0+1][iy0+1]
    z0 = z00 + ty*(z01 - z00)
    z1 = z10 + ty*(z11 - z10)
    return z0 + tx*(z1 - z0)

def _interp3d(xg, yg, zg, T, x, y, z):
    ix0 = _clamp_idx(xg, x)
    iy0 = _clamp_idx(yg, y)
    iz0 = _clamp_idx(zg, z)
    x0,x1 = xg[ix0], xg[ix0+1]
    y0,y1 = yg[iy0], yg[iy0+1]
    z0,z1 = zg[iz0], zg[iz0+1]
    tx = 0.0 if x1==x0 else (x - x0)/(x1 - x0)
    ty = 0.0 if y1==y0 else (y - y0)/(y1 - y0)
    tz = 0.0 if z1==z0 else (z - z0)/(z1 - z0)

    def Txyz(ix,iy,iz): return T[ix][iy][iz]
    c000 = Txyz(ix0,   iy0,   iz0)
    c001 = Txyz(ix0,   iy0,   iz0+1)
    c010 = Txyz(ix0,   iy0+1, iz0)
    c011 = Txyz(ix0,   iy0+1, iz0+1)
    c100 = Txyz(ix0+1, iy0,   iz0)
    c101 = Txyz(ix0+1, iy0,   iz0+1)
    c110 = Txyz(ix0+1, iy0+1, iz0)
    c111 = Txyz(ix0+1, iy0+1, iz0+1)

    c00 = c000 + tz*(c001 - c000)
    c01 = c010 + tz*(c011 - c010)
    c10 = c100 + tz*(c101 - c100)
    c11 = c110 + tz*(c111 - c110)
    c0  = c00 + ty*(c01 - c00)
    c1  = c10 + ty*(c11 - c10)
    return c0 + tx*(c1 - c0)

def _is_1d(obj):
    return isinstance(obj, (list, tuple)) and (len(obj)==0 or not isinstance(obj[0], (list, tuple)))

def _is_2d(obj):
    return (isinstance(obj, (list, tuple)) and len(obj)>0 and
            isinstance(obj[0], (list, tuple)) and
            (len(obj[0])==0 or not isinstance(obj[0][0], (list, tuple))))

def _is_3d(obj):
    return (isinstance(obj, (list, tuple)) and len(obj)>0 and
            isinstance(obj[0], (list, tuple)) and len(obj[0])>0 and
            isinstance(obj[0][0], (list, tuple)))

# ---------- main loader ----------
class F16AeroDB:
    """
    Flexible reader for F-16 aero JSON.
    Tries many alias key names and searches nested dicts.
    Exposes:
      cl(alpha_deg, mach, de_deg), cd(alpha_deg, mach), cm(alpha_deg, mach, de_deg)
    Grids must be in degrees & Mach.
    """
    # Alias listlerini geniş tuttuk
    _ALPHA_KEYS = ["alpha_grid","AoA_grid","alpha","alphas","AoA_deg","Alpha","Alpha_deg","alphaDeg","AlphaVec"]
    _MACH_KEYS  = ["mach_grid","Mach_grid","mach","Mach","M","MachVec","machVec"]
    _DE_KEYS    = ["de_grid","delta_e_grid","delta_e_deg","elevator_deg","de","delta_e","ElevatorVec","deltaE_deg"]

    _CL_KEYS = ["CL_table","CL","Cl","cL"]
    _CD_KEYS = ["CD_table","CD","Cd","cD"]
    _CM_KEYS = ["Cm_table","Cm","CM","cm"]

    _ROOT_CANDIDATES = ["f16_AerodynamicData","F16_AerodynamicData","aero","data","F16_Aero","F16"]

    def __init__(self, json_path: str):
        with open(json_path, "r", encoding="utf-8") as f:
            self.db = json.load(f)

        root = self.db
        # olası üst kapsayıcıyı bul
        for k in self._ROOT_CANDIDATES:
            if isinstance(root, Mapping) and k in root and isinstance(root[k], Mapping):
                root = root[k]
                break
        self.root = root

        # tabloları (nested dahil) bul
        self.CL_table = _deep_find_any(self.root, self._CL_KEYS)
        self.CD_table = _deep_find_any(self.root, self._CD_KEYS)
        self.Cm_table = _deep_find_any(self.root, self._CM_KEYS)

        if self.CD_table is None and self.CL_table is None and self.Cm_table is None:
            keys = _flatten_keys(self.root)
            raise KeyError("No aero tables found. Tried keys: "
                           f"CL:{self._CL_KEYS}, CD:{self._CD_KEYS}, Cm:{self._CM_KEYS}\n"
                           f"Available paths sample: {keys[:50]} ...")

        # gridleri bul (nested dahil)
        self.alpha_grid = _deep_find_any(self.root, self._ALPHA_KEYS)
        self.mach_grid  = _deep_find_any(self.root, self._MACH_KEYS)
        self.de_grid    = _deep_find_any(self.root, self._DE_KEYS)

        # Eğer gridleri bulamazsak, bazı JSON’larda gridler tablonun yanında olabilir:
        # örn: {"CL": {"alpha": [...], "mach":[...], "table":[[...]]}}
        if self.alpha_grid is None:
            self.alpha_grid = _deep_find_any(self.CL_table, self._ALPHA_KEYS) if isinstance(self.CL_table, Mapping) else None
        if self.mach_grid is None:
            self.mach_grid  = _deep_find_any(self.CL_table, self._MACH_KEYS)  if isinstance(self.CL_table, Mapping) else None
        if self.de_grid is None:
            self.de_grid    = _deep_find_any(self.CL_table, self._DE_KEYS)    if isinstance(self.CL_table, Mapping) else None

        # Bazı şemalarda tablo, alt anahtar "table"/"values" içerir
        if isinstance(self.CL_table, Mapping):
            self.CL_table = self.CL_table.get("table", self.CL_table.get("values", self.CL_table))
        if isinstance(self.CD_table, Mapping):
            self.CD_table = self.CD_table.get("table", self.CD_table.get("values", self.CD_table))
        if isinstance(self.Cm_table, Mapping):
            self.Cm_table = self.Cm_table.get("table", self.Cm_table.get("values", self.Cm_table))

        # minimum gereksinimler
        if self.alpha_grid is None:
            keys = _flatten_keys(self.root)
            raise KeyError(f"Missing alpha grid. Tried keys: {self._ALPHA_KEYS}\n"
                           f"Available paths sample: {keys[:50]} ...")
        if self.mach_grid is None and (self._has_2d(self.CL_table) or self._has_2d(self.CD_table) or self._has_2d(self.Cm_table)):
            keys = _flatten_keys(self.root)
            raise KeyError(f"Missing mach grid for 2D tables. Tried keys: {self._MACH_KEYS}\n"
                           f"Available paths sample: {keys[:50]} ...")

        # gridlerin sıralı olduğundan emin ol
        self._assert_sorted(self.alpha_grid, "alpha_grid")
        if self.mach_grid is not None:
            self._assert_sorted(self.mach_grid, "mach_grid")
        if self.de_grid is not None:
            self._assert_sorted(self.de_grid, "de_grid")

    # --- helpers ---
    @staticmethod
    def _has_2d(tab):
        return _is_2d(tab) or _is_3d(tab)

    @staticmethod
    def _assert_sorted(arr, name):
        if not isinstance(arr, Sequence) or isinstance(arr, (str, bytes)):
            raise ValueError(f"{name} must be a 1D sequence (list/tuple).")
        if len(arr) >= 2 and any(arr[i] > arr[i+1] for i in range(len(arr)-1)):
            raise ValueError(f"{name} must be ascending")

    # -------------- public API --------------
    def cl(self, alpha_deg: float, mach: float, de_deg: float = 0.0) -> float:
        tab = self.CL_table
        if tab is None:
            return 0.0
        if _is_3d(tab) and self.de_grid is not None and self.mach_grid is not None:
            return _interp3d(self.alpha_grid, self.mach_grid, self.de_grid, tab, alpha_deg, mach, de_deg)
        if _is_2d(tab) and self.mach_grid is not None:
            return _interp2d(self.alpha_grid, self.mach_grid, tab, alpha_deg, mach)
        if _is_1d(tab):
            return _interp1d(self.alpha_grid, tab, alpha_deg)
        raise ValueError("CL_table has unsupported shape")

    def cd(self, alpha_deg: float, mach: float) -> float:
        tab = self.CD_table
        if tab is None:
            CL = self.cl(alpha_deg, mach, 0.0)
            CD0, k = 0.02, 0.07
            return CD0 + k*(CL**2)
        if _is_2d(tab) and self.mach_grid is not None:
            return _interp2d(self.alpha_grid, self.mach_grid, tab, alpha_deg, mach)
        if _is_1d(tab):
            return _interp1d(self.alpha_grid, tab, alpha_deg)
        raise ValueError("CD_table has unsupported shape")

    def cm(self, alpha_deg: float, mach: float, de_deg: float = 0.0) -> float:
        tab = self.Cm_table
        if tab is None:
            # fallback linear
            return -0.02*alpha_deg - 0.01*de_deg
        if _is_3d(tab) and self.de_grid is not None and self.mach_grid is not None:
            return _interp3d(self.alpha_grid, self.mach_grid, self.de_grid, tab, alpha_deg, mach, de_deg)
        if _is_2d(tab) and self.mach_grid is not None:
            return _interp2d(self.alpha_grid, self.mach_grid, tab, alpha_deg, mach)
        if _is_1d(tab):
            return _interp1d(self.alpha_grid, tab, alpha_deg)
        raise ValueError("Cm_table has unsupported shape")
