# main.py
# High-level simulation loop:
#  - Load aero DB
#  - Trim at (V_trim, h_trim)
#  - Integrate with RK4
#  - (Optional) Send pose to FlightGear via UDP

import math
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent / "src"))

from src.f16_constants import F16_CONSTANTS as C
from src.f16_aero_loader import F16AeroDB
from src.f16_dynamics import f_dot, trim_level_flight
from src.f16_kinematics import integrate_position
from src.f16_forces import uvw_to_alphabeta

# ---- Optional FlightGear UDP sender (enable by set SEND_TO_FG=True)
import socket
SEND_TO_FG = True

class FGSender:
    """Minimal UDP sender for FlightGear 'myproto' (lat,lon,alt,roll,pitch,heading)."""
    def __init__(self, host="127.0.0.1", port=5500):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_pose(self, lat_deg, lon_deg, alt_m, roll_deg, pitch_deg, heading_deg):
        line = f"{lat_deg:.8f},{lon_deg:.8f},{alt_m:.2f},{roll_deg:.3f},{pitch_deg:.3f},{heading_deg:.3f}\n"
        self.sock.sendto(line.encode("ascii"), self.addr)

# --- If you already have geo_utils.ned_to_geodetic(), import it; else a quick fallback:
try:
    from geo_utils import ned_to_geodetic
except Exception:
    # Very small-area approximation around (lat0, lon0)
    import math
    def ned_to_geodetic(N, E, D, lat0_deg, lon0_deg, h0_m):
        dlat = (N / 111_320.0)
        dlon = (E / (111_320.0 * math.cos(math.radians(lat0_deg))))
        return (lat0_deg + dlat, lon0_deg + dlon, h0_m - D)


# --------------- Simulation settings -----------------
DT       = 0.01         # time step [s]
T_FINAL  = 10.0         # total time [s]
N_STEPS  = int(T_FINAL / DT)
LAT0_DEG = 41.015137    # reference geodetic (e.g. Istanbul)
LON0_DEG = 28.979530
LEF_ON   = False
SBRAKE   = 0.0

# FlightGear output
fg = FGSender() if SEND_TO_FG else None

# --------------- Load Aero DB ------------------------
DB_PATH = Path(__file__).parent / "data" / "F16_database.json"
db = F16AeroDB(str(DB_PATH))

# --------------- Trim at (V_trim, h_trim) ------------
V0 = float(C["V_trim"])
h0 = float(C["h_trim"])

trim = trim_level_flight(V0, h0, db, lef=LEF_ON, speedbrake=SBRAKE, lat_deg=LAT0_DEG)
state    = trim["state0"].copy()
controls = trim["controls0"].copy()
thrust_N = trim["thrust_N"]

print(f"[TRIM] theta={math.degrees(trim['theta']):.2f} deg | "
      f"de={controls['de']:.2f} deg | T={thrust_N:.0f} N | "
      f"alpha≈{trim['alpha_deg']:.2f} deg")

# --------------- Initial position (NED) --------------
N = 0.0; E = 0.0; D = -h0   # start right above the reference point

# --------------- RK4 Integrator ----------------------
def rk4_step(state, controls, dt):
    KEYS = ("u","v","w","p","q","r","phi","theta","psi","h")

    def splus(s, k, h):
        t = s.copy()
        for key in KEYS:
            t[key] = t[key] + h * k[f"{key}_dot"]
        return t

    k1 = f_dot(state, controls, db, thrust_N, lef=LEF_ON, speedbrake=SBRAKE, lat_deg=LAT0_DEG)
    k2 = f_dot(splus(state, k1, dt/2), controls, db, thrust_N, lef=LEF_ON, speedbrake=SBRAKE, lat_deg=LAT0_DEG)
    k3 = f_dot(splus(state, k2, dt/2), controls, db, thrust_N, lef=LEF_ON, speedbrake=SBRAKE, lat_deg=LAT0_DEG)
    k4 = f_dot(splus(state, k3, dt),   controls, db, thrust_N, lef=LEF_ON, speedbrake=SBRAKE, lat_deg=LAT0_DEG)

    for key in KEYS:
        state[key] += (dt/6.0) * (
            k1[f"{key}_dot"] + 2.0*k2[f"{key}_dot"] + 2.0*k3[f"{key}_dot"] + k4[f"{key}_dot"]
        )
    return state


# --------------- Main loop ---------------------------
t = 0.0
for step in range(N_STEPS):
    # 1) integrate dynamics
    state = rk4_step(state, controls, DT)

    # 2) integrate position in NED from body velocity
    N, E, D = integrate_position(N, E, D,
                                 state["u"], state["v"], state["w"],
                                 state["phi"], state["theta"], state["psi"], DT)
    state["h"] = -D

    # 3) Send to FlightGear (optional)
    if fg is not None:
        lat_deg, lon_deg, alt_m = ned_to_geodetic(N, E, D, LAT0_DEG, LON0_DEG, h0)
        roll_deg  = math.degrees(state["phi"])
        pitch_deg = math.degrees(state["theta"])
        heading   = (math.degrees(state["psi"]) % 360.0)
        fg.send_pose(lat_deg, lon_deg, alt_m, roll_deg, pitch_deg, heading)

    # 4) Console log at 1 Hz
    if (step % int(1.0/DT)) == 0:
        alpha_deg, beta_deg, V_now = uvw_to_alphabeta(state["u"], state["v"], state["w"])
        print(f"t={t:4.1f}s | V={V_now:6.2f} m/s | alpha={alpha_deg:5.2f} deg | "
              f"theta={math.degrees(state['theta']):6.2f} deg | h={state['h']:7.1f} m")
    t += DT

print("\nDone. Final state:")
alpha_deg, beta_deg, V_now = uvw_to_alphabeta(state["u"], state["v"], state["w"])
print(f"V={V_now:.2f} m/s, alpha={alpha_deg:.2f} deg, theta={math.degrees(state['theta']):.2f} deg, h={state['h']:.1f} m")
print(f"N={N:.1f} m, E={E:.1f} m, D={D:.1f} m")
