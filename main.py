# main.py
# High-level simulation loop:
#  - Load aero DB
#  - Trim at (V_trim, h_trim)
#  - Integrate with RK4
#  - (Optional) Send pose to FlightGear via UDP

import math
import numpy as np
import sys
import time
from pathlib import Path
from collections import deque

# Add src to path
sys.path.insert(0, str(Path(__file__).parent / "src"))

from src.f16_constants import F16_CONSTANTS as C
from src.f16_aero_loader import F16AeroDB
from src.f16_dynamics import f_dot, trim_level_flight
from src.f16_kinematics import integrate_position, ned_dot_from_body, euler_to_quat, quat_to_euler, ned_dot_from_body_quat, quat_normalize
from src.f16_forces import uvw_to_alphabeta

# ---- Optional FlightGear UDP sender (enable by set SEND_TO_FG=True)
import socket
SEND_TO_FG = True

# Optional live dashboard (matplotlib)
try:
    import matplotlib.pyplot as plt
    HAS_MPL = True
except Exception:
    HAS_MPL = False

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

# --------------- Dashboard settings ------------------
ENABLE_DASHBOARD = True and HAS_MPL
DASH_RATE = 10.0  # Hz
DASH_UPDATE_INTERVAL = max(1, int(1.0 / (DASH_RATE * DT)))
PLOT_WINDOW_SEC = 20.0  # seconds of data to keep in plots

# --------------- Live Dashboard ------------------
class LiveDashboard:
    def __init__(self, window_sec, dt):
        self.window_sec = float(window_sec)
        nmax = int(window_sec / dt)
        self.t = deque(maxlen=nmax)
        self.alpha = deque(maxlen=nmax)
        self.beta = deque(maxlen=nmax)
        self.V = deque(maxlen=nmax)
        self.p = deque(maxlen=nmax); self.q = deque(maxlen=nmax); self.r = deque(maxlen=nmax)
        self.roll = deque(maxlen=nmax); self.pitch = deque(maxlen=nmax); self.heading = deque(maxlen=nmax)
        self.h = deque(maxlen=nmax)
        self.de = deque(maxlen=nmax); self.da = deque(maxlen=nmax); self.dr = deque(maxlen=nmax)

        if HAS_MPL:
            plt.ion()
            self.fig, self.ax = plt.subplots(3, 2, figsize=(11, 8), constrained_layout=True)

            self.lines = {}
            self.lines['alpha'], = self.ax[0,0].plot([], [], label='alpha [deg]')
            self.lines['beta'],  = self.ax[0,0].plot([], [], label='beta [deg]')
            self.ax[0,0].set_title('Alpha/Beta'); self.ax[0,0].legend(loc='upper right'); self.ax[0,0].grid(True)
            self.lines['V'], = self.ax[0,1].plot([], [], label='V [m/s]')
            self.ax[0,1].set_title('Airspeed'); self.ax[0,1].grid(True)
            self.lines['p'], = self.ax[1,0].plot([], [], label='p [rad/s]')
            self.lines['q'], = self.ax[1,0].plot([], [], label='q [rad/s]')
            self.lines['r'], = self.ax[1,0].plot([], [], label='r [rad/s]')
            self.ax[1,0].set_title('Body rates'); self.ax[1,0].legend(loc='upper right'); self.ax[1,0].grid(True)
            self.lines['roll'],  = self.ax[1,1].plot([], [], label='roll φ [deg]')
            self.lines['pitch'], = self.ax[1,1].plot([], [], label='pitch θ [deg]')
            self.lines['head'],  = self.ax[1,1].plot([], [], label='heading ψ [deg]')
            self.ax[1,1].set_title('Euler angles'); self.ax[1,1].legend(loc='upper right'); self.ax[1,1].grid(True)
            self.lines['h'], = self.ax[2,0].plot([], [], label='h [m]')
            self.ax[2,0].set_title('Altitude'); self.ax[2,0].grid(True)
            self.lines['de'], = self.ax[2,1].plot([], [], label='de [deg]')
            self.lines['da'], = self.ax[2,1].plot([], [], label='da [deg]')
            self.lines['dr'], = self.ax[2,1].plot([], [], label='dr [deg]')
            self.ax[2,1].set_title('Controls'); self.ax[2,1].legend(loc='upper right'); self.ax[2,1].grid(True)

    def append(self, t, state, controls):
        a_deg, b_deg, V_now = uvw_to_alphabeta(state["u"], state["v"], state["w"])
        self.t.append(t)
        self.alpha.append(a_deg); self.beta.append(b_deg); self.V.append(V_now)
        self.p.append(state['p']); self.q.append(state['q']); self.r.append(state['r'])
        self.roll.append(math.degrees(state['phi']))
        self.pitch.append(math.degrees(state['theta']))
        self.heading.append((math.degrees(state['psi']) % 360.0))
        self.h.append(state['h'])
        self.de.append(controls.get('de', 0.0)); self.da.append(controls.get('da', 0.0)); self.dr.append(controls.get('dr', 0.0))

    def draw(self):
        if not HAS_MPL:
            return
        t = list(self.t)
        if not t:
            return
        def upd(key, y):
            self.lines[key].set_data(t, y)
        upd('alpha', list(self.alpha)); upd('beta', list(self.beta))
        upd('V', list(self.V))
        upd('p', list(self.p)); upd('q', list(self.q)); upd('r', list(self.r))
        upd('roll', list(self.roll)); upd('pitch', list(self.pitch)); upd('head', list(self.heading))
        upd('h', list(self.h))
        upd('de', list(self.de)); upd('da', list(self.da)); upd('dr', list(self.dr))

        # X-axis window: last window_sec seconds; avoid identical limits when only 1 sample
        t_last = t[-1]
        tmin = t_last - self.window_sec
        tmax = t_last
        if tmax - tmin < 1e-6:
            tmin = t_last - 1e-3
            tmax = t_last + 1e-3
        for ax in self.ax.ravel():
            ax.set_xlim(tmin, tmax)
        for a in [self.ax[0,0], self.ax[0,1], self.ax[1,0], self.ax[1,1], self.ax[2,0], self.ax[2,1]]:
            a.relim(); a.autoscale_view()
        self.fig.canvas.draw_idle()
        try:
            self.fig.canvas.flush_events()
        except Exception:
            pass
        try:
            plt.pause(0.001)
        except Exception:
            pass
 
# --------------- Scenario settings ------------------
# Enable dynamic control inputs for maneuvers
ENABLE_MANEUVER = True   # True: inject maneuver inputs; False: pure trim (steady)
MANEUVER_TYPE = "turn"   # Options: "climb", "turn", "roll", "none"

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

# --------------- Initial position & attitude --------------
N = 0.0; E = 0.0; D = -h0   # start right above the reference point
q_att = euler_to_quat(state["phi"], state["theta"], state["psi"])  # quaternion attitude

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
# RK4 for NED position using intermediate states
def rk4_step_position_quat(N, E, D, state, q, q2, q3, q4, dt):
    # Helper to build intermediate state
    KEYS = ("u","v","w","p","q","r","phi","theta","psi","h")
    # k1..k4 position rates using quaternion attitudes
    N1, E1, D1 = ned_dot_from_body_quat(state["u"], state["v"], state["w"], q)
    # k2
    N2, E2, D2 = ned_dot_from_body_quat(state["u"], state["v"], state["w"], q2)
    # k3
    N3, E3, D3 = ned_dot_from_body_quat(state["u"], state["v"], state["w"], q3)
    # k4
    N4, E4, D4 = ned_dot_from_body_quat(state["u"], state["v"], state["w"], q4)

    N_next = N + (dt/6.0) * (N1 + 2.0*N2 + 2.0*N3 + N4)
    E_next = E + (dt/6.0) * (E1 + 2.0*E2 + 2.0*E3 + E4)
    D_next = D + (dt/6.0) * (D1 + 2.0*D2 + 2.0*D3 + D4)
    return N_next, E_next, D_next

# RK4 for quaternion attitude using body rates (p,q,r)
def rk4_step_quaternion(q, state, k1, k2, k3, k4, dt):
    # qdot = 0.5 * Omega(omega) * q
    def omega_mat(p, q_r, r):
        return np.array([
            [0.0, -p,   -q_r, -r],
            [p,    0.0,  r,  -q_r],
            [q_r, -r,    0.0,  p],
            [r,    q_r, -p,   0.0]
        ], dtype=float)
    
    def qdot(qv, p, q_r, r):
        return 0.5 * (omega_mat(p, q_r, r) @ qv)

    # Stage 1
    p1, q1, r1 = state["p"], state["q"], state["r"]
    qdot1 = qdot(q, p1, q1, r1)
    q2 = quat_normalize(q + (dt/2.0) * qdot1)

    # Stage 2 (use state + dt/2 with angular rates approximated by adding dt/2*k for rates)
    p2 = state["p"] + (dt/2.0) * k1["p_dot"]
    q2r= state["q"] + (dt/2.0) * k1["q_dot"]
    r2 = state["r"] + (dt/2.0) * k1["r_dot"]
    qdot2 = qdot(q2, p2, q2r, r2)
    q3 = quat_normalize(q + (dt/2.0) * qdot2)

    # Stage 3
    p3 = state["p"] + (dt/2.0) * k2["p_dot"]
    q3r= state["q"] + (dt/2.0) * k2["q_dot"]
    r3 = state["r"] + (dt/2.0) * k2["r_dot"]
    qdot3 = qdot(q3, p3, q3r, r3)
    q4 = quat_normalize(q + dt * qdot3)

    # Stage 4
    p4 = state["p"] + dt * k3["p_dot"]
    q4r= state["q"] + dt * k3["q_dot"]
    r4 = state["r"] + dt * k3["r_dot"]
    qdot4 = qdot(q4, p4, q4r, r4)

    q_next = quat_normalize(q + (dt/6.0) * (qdot1 + 2.0*qdot2 + 2.0*qdot3 + qdot4))
    return q_next, q2, q3, q4


# (moved above main loop)

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

dash = LiveDashboard(PLOT_WINDOW_SEC, DT) if ENABLE_DASHBOARD else None

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
    # Dynamics RK4 (collect stage derivatives for attitude/position RK4)
    KEYS = ("u","v","w","p","q","r","phi","theta","psi","h")
    def splus(s, k, h):
        t = s.copy()
        for key in KEYS:
            t[key] = t[key] + h * k[f"{key}_dot"]
        return t

    k1 = f_dot(state, controls, db, thrust_N, lef=LEF_ON, speedbrake=SBRAKE, lat_deg=LAT0_DEG)
    k2 = f_dot(splus(state, k1, DT/2), controls, db, thrust_N, lef=LEF_ON, speedbrake=SBRAKE, lat_deg=LAT0_DEG)
    k3 = f_dot(splus(state, k2, DT/2), controls, db, thrust_N, lef=LEF_ON, speedbrake=SBRAKE, lat_deg=LAT0_DEG)
    k4 = f_dot(splus(state, k3, DT),   controls, db, thrust_N, lef=LEF_ON, speedbrake=SBRAKE, lat_deg=LAT0_DEG)
    for key in KEYS:
        state[key] += (DT/6.0) * (
            k1[f"{key}_dot"] + 2.0*k2[f"{key}_dot"] + 2.0*k3[f"{key}_dot"] + k4[f"{key}_dot"]
        )

    # 2) integrate attitude quaternion (RK4)
    q_att, q2, q3, q4 = rk4_step_quaternion(q_att, state, k1, k2, k3, k4, DT)

    # 3) integrate position in NED (RK4 using quaternion stages)
    N, E, D = rk4_step_position_quat(N, E, D, state, q_att, q2, q3, q4, DT)
    state["h"] = -D
    
    # 3.3) Overwrite Euler from quaternion (for logging/FG); no manual clamping needed
    phi_q, theta_q, psi_q = quat_to_euler(q_att)
    state["phi"], state["theta"], state["psi"] = phi_q, theta_q, psi_q

    # 3) Send to FlightGear (optional) - throttled to FG_UPDATE_RATE
    if fg is not None and (step % FG_SEND_INTERVAL == 0):
        try:
            lat_deg, lon_deg, alt_m = ned_to_geodetic(N, E, D, LAT0_DEG, LON0_DEG, h0)
            roll_deg  = math.degrees(state["phi"])
            pitch_deg = math.degrees(state["theta"])
            heading   = (math.degrees(state["psi"]) % 360.0)
            roll_deg = max(-180, min(180, roll_deg))
            pitch_deg = max(-90, min(90, pitch_deg))
            heading = max(0, min(360, heading))
            alt_m = max(-1000, min(50000, alt_m))
            if (math.isfinite(roll_deg) and math.isfinite(pitch_deg) and 
                math.isfinite(heading) and math.isfinite(alt_m) and
                math.isfinite(lat_deg) and math.isfinite(lon_deg)):
                fg.send_pose(lat_deg, lon_deg, alt_m, roll_deg, pitch_deg, heading)
        except Exception as e:
            if step % int(10.0/DT) == 0:
                print(f"[UDP Error] {str(e)}")

    # 3.5) Dashboard update (throttled)
    if dash is not None and (step % DASH_UPDATE_INTERVAL == 0):
        dash.append(t, state, controls)
        dash.draw()

    # 4) Advance sim time
    t += DT


print("\nDone. Final state:")
alpha_deg, beta_deg, V_now = uvw_to_alphabeta(state["u"], state["v"], state["w"])
print(f"V={V_now:.2f} m/s, alpha={alpha_deg:.2f} deg, theta={math.degrees(state['theta']):.2f} deg, h={state['h']:.1f} m")
print(f"N={N:.1f} m, E={E:.1f} m, D={D:.1f} m")
