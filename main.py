# main.py
# High-level simulation loop:
#  - Load aero DB
#  - Trim at (V_trim, h_trim)
#  - Integrate with RK4
#  - (Optional) Send pose to FlightGear via UDP

import math
import sys
import time
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
        self.sock.setblocking(False)  # Non-blocking UDP (prevents stalls)

    def send_pose(self, lat_deg, lon_deg, alt_m, roll_deg, pitch_deg, heading_deg):
        try:
            line = f"{lat_deg:.8f},{lon_deg:.8f},{alt_m:.2f},{roll_deg:.3f},{pitch_deg:.3f},{heading_deg:.3f}\n"
            self.sock.sendto(line.encode("ascii"), self.addr)
        except BlockingIOError:
            pass  # Skip if buffer full (non-critical)

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
T_FINAL  = 60.0         # total time [s] (1 minute simulation)
N_STEPS  = int(T_FINAL / DT)
LAT0_DEG = 41.015137    # reference geodetic (e.g. Istanbul)
LON0_DEG = 28.979530
LEF_ON   = False
SBRAKE   = 0.0

# --------------- Real-time synchronization -----------
REAL_TIME = True        # If True, synchronize with real-time (slower but matches FlightGear)
                        # If False, run as fast as possible
TIME_SCALE = 1.0        # Time multiplier (1.0 = real-time, 2.0 = 2x speed, 0.5 = half speed)
RT_SYNC_INTERVAL = 10   # Check real-time sync every N steps (reduce overhead)

# --------------- FlightGear output rate -------------
FG_UPDATE_RATE = 30.0   # Hz - Update rate for FlightGear (30 Hz is sufficient, reduces CPU)
FG_SEND_INTERVAL = max(1, int(1.0 / (FG_UPDATE_RATE * DT)))  # Steps between sends

# --------------- Scenario settings ------------------
# Enable dynamic control inputs for maneuvers
ENABLE_MANEUVER = False  # Set to False for steady flight (start with steady to debug)
MANEUVER_TYPE = "none"   # Options: "climb", "turn", "roll", "none"

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

alpha_trim, beta_trim, V_trim_check = uvw_to_alphabeta(trim["state0"]["u"], 
                                                        trim["state0"]["v"], 
                                                        trim["state0"]["w"])
print(f"[TRIM] theta={math.degrees(trim['theta']):.2f} deg | "
      f"de={controls['de']:.2f} deg | T={thrust_N:.0f} N | "
      f"alpha_trim={trim['alpha_deg']:.3f} deg | alpha_calc={alpha_trim:.3f} deg | "
      f"u={trim['state0']['u']:.2f} v={trim['state0']['v']:.2f} w={trim['state0']['w']:.2f}")
print(f"[SIM] Duration: {T_FINAL:.1f} s | Time step: {DT} s | Maneuver: {MANEUVER_TYPE if ENABLE_MANEUVER else 'none'}")
print(f"[SIM] Real-time: {'ON' if REAL_TIME else 'OFF'} | Time scale: {TIME_SCALE}x | FG rate: {FG_UPDATE_RATE} Hz")

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


# --------------- Dynamic control function -------------
def update_controls(t, controls_base, maneuver_type):
    """Update control inputs based on time and maneuver type."""
    controls = controls_base.copy()
    
    if not ENABLE_MANEUVER or maneuver_type == "none":
        return controls
    
    # Example maneuvers (adjust as needed)
    if maneuver_type == "climb":
        # Gentle climb: small elevator input, limited duration
        if t > 5.0 and t < 15.0:
            # Small climb input (0.5 deg max)
            controls["de"] = controls_base["de"] + 0.5 * (1.0 - math.exp(-(t-5.0)/2.0))
        elif t >= 15.0 and t < 25.0:
            # Return to level flight gradually
            controls["de"] = controls_base["de"] + 0.5 * math.exp(-(t-15.0)/2.0)
        else:
            # Keep trim controls
            controls["de"] = controls_base["de"]
    
    elif maneuver_type == "turn":
        # Coordinated turn: aileron and rudder
        if t > 5.0 and t < 25.0:
            controls["da"] = 5.0 * math.sin(2.0 * math.pi * (t - 5.0) / 20.0)
            controls["dr"] = 2.0 * math.sin(2.0 * math.pi * (t - 5.0) / 20.0)
        elif t > 45.0:
            # Return to level
            controls["da"] = 0.0
            controls["dr"] = 0.0
    
    elif maneuver_type == "roll":
        # Roll maneuver: aileron only
        if t > 5.0 and t < 15.0:
            controls["da"] = 10.0 * math.sin(4.0 * math.pi * (t - 5.0) / 10.0)
        elif t > 15.0:
            controls["da"] = 0.0
    
    return controls


# --------------- Main loop ---------------------------
t = 0.0
controls_base = controls.copy()  # Save base trim controls

# Real-time synchronization
if REAL_TIME:
    loop_start_time = time.time()
    print("[INFO] Running in REAL-TIME mode - simulation will match wall-clock time")

for step in range(N_STEPS):
    # Real-time synchronization (only check periodically to reduce overhead)
    if REAL_TIME and step % RT_SYNC_INTERVAL == 0:
        expected_time = step * DT / TIME_SCALE
        actual_time = time.time() - loop_start_time
        sleep_time = expected_time - actual_time
        if sleep_time > 0.001:  # Only sleep if significant delay needed
            time.sleep(min(sleep_time, 0.1))  # Cap sleep time to prevent large jumps
    
    # Update controls for maneuvers
    if ENABLE_MANEUVER:
        controls = update_controls(t, controls_base, MANEUVER_TYPE)
    
    # 1) integrate dynamics
    state = rk4_step(state, controls, DT)

    # 2) integrate position in NED from body velocity
    N, E, D = integrate_position(N, E, D,
                                 state["u"], state["v"], state["w"],
                                 state["phi"], state["theta"], state["psi"], DT)
    state["h"] = -D
    
    # 2.3) Normalize Euler angles to reasonable ranges
    # Wrap angles to prevent unbounded growth
    state["phi"] = ((state["phi"] + math.pi) % (2*math.pi)) - math.pi  # -180 to 180 deg
    # Pitch should be clamped, not wrapped (aircraft can't exceed ±90 deg physically)
    if state["theta"] > math.pi/2:
        state["theta"] = math.pi/2 - 0.01  # Clamp to slightly below 90 deg
    elif state["theta"] < -math.pi/2:
        state["theta"] = -math.pi/2 + 0.01  # Clamp to slightly above -90 deg
    state["psi"] = (state["psi"] % (2*math.pi))  # 0 to 360 deg

    # 3) Send to FlightGear (optional) - throttled to FG_UPDATE_RATE
    if fg is not None and (step % FG_SEND_INTERVAL == 0):
        try:
            lat_deg, lon_deg, alt_m = ned_to_geodetic(N, E, D, LAT0_DEG, LON0_DEG, h0)
            # FlightGear external FDM conventions (standard aviation):
            # Roll: positive = right wing down (matches our phi)
            # Pitch: positive = nose up (standard - should match our theta)
            # Heading: 0-360 degrees clockwise from North (matches our psi)
            roll_deg  = math.degrees(state["phi"])
            pitch_deg = math.degrees(state["theta"])
            heading   = (math.degrees(state["psi"]) % 360.0)
            
            # Safety checks - clamp values to reasonable ranges
            roll_deg = max(-180, min(180, roll_deg))
            pitch_deg = max(-90, min(90, pitch_deg))
            heading = max(0, min(360, heading))
            alt_m = max(-1000, min(50000, alt_m))  # Reasonable altitude range
            
            # Check for NaN/Inf
            if not (math.isfinite(roll_deg) and math.isfinite(pitch_deg) and 
                    math.isfinite(heading) and math.isfinite(alt_m) and
                    math.isfinite(lat_deg) and math.isfinite(lon_deg)):
                print(f"[ERROR] Invalid values at t={t:.2f}s: roll={roll_deg}, pitch={pitch_deg}")
                continue
            
            # Altitude in meters (myproto.xml typically expects meters for external FDM)
            fg.send_pose(lat_deg, lon_deg, alt_m, roll_deg, pitch_deg, heading)
        except Exception as e:
            if step % int(10.0/DT) == 0:  # Log error occasionally
                print(f"[UDP Error] {str(e)}")

    # 4) Console log at 1 Hz
    if (step % int(1.0/DT)) == 0:
        alpha_deg, beta_deg, V_now = uvw_to_alphabeta(state["u"], state["v"], state["w"])
        # Debug warning for unusual alpha values
        if abs(alpha_deg) > 90:
            print(f"[WARNING] t={t:.2f}s: alpha={alpha_deg:.4f} deg (unusual!)")
        if REAL_TIME:
            elapsed = time.time() - loop_start_time
            print(f"t={t:4.1f}s (real: {elapsed:.1f}s) | V={V_now:6.2f} m/s | alpha={alpha_deg:6.3f} deg | "
                  f"theta={math.degrees(state['theta']):6.2f} deg | h={state['h']:7.1f} m")
        else:
            print(f"t={t:4.1f}s | V={V_now:6.2f} m/s | alpha={alpha_deg:6.3f} deg | "
                  f"theta={math.degrees(state['theta']):6.2f} deg | h={state['h']:7.1f} m")
    
    t += DT

print("\nDone. Final state:")
alpha_deg, beta_deg, V_now = uvw_to_alphabeta(state["u"], state["v"], state["w"])
print(f"V={V_now:.2f} m/s, alpha={alpha_deg:.2f} deg, theta={math.degrees(state['theta']):.2f} deg, h={state['h']:.1f} m")
print(f"N={N:.1f} m, E={E:.1f} m, D={D:.1f} m")
