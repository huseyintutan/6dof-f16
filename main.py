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
import gc
from pathlib import Path
from collections import deque

# Add src to path
sys.path.insert(0, str(Path(__file__).parent / "src"))

from src.f16_constants import F16_CONSTANTS as C
from src.f16_aero_loader import F16AeroDB
from src.f16_dynamics import f_dot, trim_level_flight
from src.f16_kinematics import integrate_position, ned_dot_from_body, euler_to_quat, quat_to_euler, ned_dot_from_body_quat, quat_normalize
from src.f16_forces import uvw_to_alphabeta
from src.f16_sensors import SensorSuite

# Sensor fusion (EKF) - optional
try:
    from src.sensor_fusion_ekf import EKFSensorFusion
    HAS_EKF = True
except ImportError as e:
    HAS_EKF = False
    print(f"[WARN] EKF sensor fusion not available: {e}")

# EKF Dashboard - optional
try:
    from src.ekf_dashboard import EKFDashboard, HAS_FLASK as HAS_EKF_FLASK
    HAS_EKF_DASHBOARD = HAS_EKF_FLASK
except ImportError as e:
    HAS_EKF_DASHBOARD = False
    print(f"[WARN] EKF dashboard not available: {e}")

# Web dashboard (optional)
try:
    from src.web_dashboard import WebDashboard, HAS_FLASK
    HAS_WEB_DASHBOARD = HAS_FLASK
    if not HAS_FLASK:
        print("[WARN] Flask not available. Install with: pip install flask flask-socketio")
except ImportError as e:
    HAS_WEB_DASHBOARD = False
    print(f"[WARN] Web dashboard not available: {e}")
    print("[WARN] Install with: pip install flask flask-socketio")

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
        # Set socket buffer size for better performance
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)  # 64KB buffer
        except Exception:
            pass  # Ignore if not supported
        # Pre-allocate message buffer to avoid repeated string formatting overhead
        self.message_buffer = bytearray(100)  # Pre-allocated buffer
        print(f"[UDP] FlightGear sender initialized (buffer: 64KB)")

    def send_pose(self, lat_deg, lon_deg, alt_m, roll_deg, pitch_deg, heading_deg):
        try:
            # Optimized string formatting (fewer decimal places for better performance)
            line = f"{lat_deg:.6f},{lon_deg:.6f},{alt_m:.1f},{roll_deg:.2f},{pitch_deg:.2f},{heading_deg:.2f}\n"
            self.sock.sendto(line.encode("ascii"), self.addr)
        except (BlockingIOError, OSError):
            pass  # Skip if buffer full or socket error (non-critical)

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
DT       = 0.01         # time step [s] (100 Hz internal rate)
T_FINAL  = 300.0        # total time [s] (5 minutes simulation - longer for EKF testing)
N_STEPS  = int(T_FINAL / DT)  # Total: 6,000 steps
LAT0_DEG = 41.015137    # reference geodetic (e.g. Istanbul)
LON0_DEG = 28.979530
LEF_ON   = False
SBRAKE   = 0.0

# --------------- Real-time synchronization -----------
REAL_TIME = True        # If True, synchronize with real-time (matches FlightGear)
                        # If False, run as fast as possible (faster but no real-time sync)
TIME_SCALE = 1.0        # Time multiplier (1.0 = real-time, 2.0 = 2x speed, 0.5 = half speed)
RT_SYNC_INTERVAL = 20   # Check real-time sync every N steps (increased for better performance)

# --------------- FlightGear output rate -------------
FG_UPDATE_RATE = 10.0   # Hz - Reduced from 30 Hz for better performance (10 Hz is sufficient for visualization)
FG_SEND_INTERVAL = max(1, int(1.0 / (FG_UPDATE_RATE * DT)))  # Steps between sends

# --------------- Dashboard settings ------------------
ENABLE_DASHBOARD = False and HAS_MPL  # Disabled by default (use web dashboard instead)
DASH_RATE = 2.0  # Hz - Further reduced for better performance (2 Hz is sufficient for visualization)
DASH_UPDATE_INTERVAL = max(1, int(1.0 / (DASH_RATE * DT)))
PLOT_WINDOW_SEC = 20.0  # seconds of data to keep in plots

# --------------- Web Dashboard settings ------------------
ENABLE_WEB_DASHBOARD = True and HAS_WEB_DASHBOARD  # Enable web dashboard (RECOMMENDED - much lower CPU)
WEB_DASHBOARD_PORT = 5000  # HTTP server port
WEB_DASHBOARD_RATE = 2.0  # Hz - Update rate for web dashboard
WEB_DASHBOARD_INTERVAL = max(1, int(1.0 / (WEB_DASHBOARD_RATE * DT)))

# --------------- Sensor Dashboard settings ------------------
ENABLE_SENSOR_DASHBOARD = False and HAS_MPL  # Disabled (use web dashboard instead)
SENSOR_DASH_RATE = 2.0  # Hz - Further reduced for better performance
SENSOR_DASH_UPDATE_INTERVAL = max(1, int(1.0 / (SENSOR_DASH_RATE * DT)))
COMBINE_DASHBOARDS = True  # Combine both dashboards in single window (RECOMMENDED)

# --------------- Sensor settings ------------------
ENABLE_SENSORS = True          # Enable sensor modeling
SENSOR_NOISE = True            # Enable sensor noise
SENSOR_BIAS = True             # Enable sensor bias
SENSOR_LOG_RATE = 1.0          # Hz - Rate to print sensor outputs to console
SENSOR_LOG_INTERVAL = max(1, int(1.0 / (SENSOR_LOG_RATE * DT)))

# --------------- Sensor Fusion (EKF) settings ------------------
ENABLE_SENSOR_FUSION = True    # Enable EKF sensor fusion
SENSOR_FUSION_RATE = 10.0      # Hz - EKF update rate
SENSOR_FUSION_INTERVAL = max(1, int(1.0 / (SENSOR_FUSION_RATE * DT)))

# --------------- EKF Dashboard settings ------------------
ENABLE_EKF_DASHBOARD = True and HAS_EKF_DASHBOARD  # Enable EKF dashboard (separate port)
EKF_DASHBOARD_PORT = 5001  # EKF dashboard port (different from main dashboard)

# --------------- System Resource Pre-allocation ------------------
def preallocate_resources():
    """Pre-allocate system resources for better performance."""
    # Disable automatic garbage collection during critical simulation
    gc.disable()
    
    # Pre-allocate numpy arrays for common operations
    # This reduces memory fragmentation
    _temp_arrays = {
        'state_vector': np.zeros(10, dtype=np.float64),
        'quaternion': np.zeros(4, dtype=np.float64),
        'body_rates': np.zeros(3, dtype=np.float64),
        'ned_velocity': np.zeros(3, dtype=np.float64),
    }
    
    # Force memory allocation
    for arr in _temp_arrays.values():
        arr.fill(0.0)
    
    print("[RESOURCE] Pre-allocated system resources")
    return _temp_arrays

# --------------- Combined Dashboard (State + Sensors) ------------------
class CombinedDashboard:
    """Combined dashboard showing both state and sensor data in a single window."""
    def __init__(self, window_sec, dt, include_sensors=True):
        self.window_sec = float(window_sec)
        self.include_sensors = include_sensors
        nmax = int(window_sec / dt)
        
        # Pre-allocate all deques with zeros to avoid dynamic resizing
        print(f"[DASHBOARD] Pre-allocating {nmax} data points per variable...")
        
        # State data - pre-allocate with zeros
        self.t = deque([0.0] * nmax, maxlen=nmax)
        self.alpha = deque([0.0] * nmax, maxlen=nmax)
        self.beta = deque([0.0] * nmax, maxlen=nmax)
        self.V = deque([0.0] * nmax, maxlen=nmax)
        self.p = deque([0.0] * nmax, maxlen=nmax)
        self.q = deque([0.0] * nmax, maxlen=nmax)
        self.r = deque([0.0] * nmax, maxlen=nmax)
        self.roll = deque([0.0] * nmax, maxlen=nmax)
        self.pitch = deque([0.0] * nmax, maxlen=nmax)
        self.heading = deque([0.0] * nmax, maxlen=nmax)
        self.h = deque([0.0] * nmax, maxlen=nmax)
        self.de = deque([0.0] * nmax, maxlen=nmax)
        self.da = deque([0.0] * nmax, maxlen=nmax)
        self.dr = deque([0.0] * nmax, maxlen=nmax)
        
        # Sensor data (if enabled) - pre-allocate with zeros
        if include_sensors:
            self.accel_fx = deque([0.0] * nmax, maxlen=nmax)
            self.accel_fy = deque([0.0] * nmax, maxlen=nmax)
            self.accel_fz = deque([0.0] * nmax, maxlen=nmax)
            self.gyro_p = deque([0.0] * nmax, maxlen=nmax)
            self.gyro_q = deque([0.0] * nmax, maxlen=nmax)
            self.gyro_r = deque([0.0] * nmax, maxlen=nmax)
            self.gps_alt = deque([0.0] * nmax, maxlen=nmax)
            self.air_V = deque([0.0] * nmax, maxlen=nmax)
            self.air_alpha = deque([0.0] * nmax, maxlen=nmax)
            self.air_beta = deque([0.0] * nmax, maxlen=nmax)
            self.mag_heading = deque([0.0] * nmax, maxlen=nmax)
            self.baro_alt = deque([0.0] * nmax, maxlen=nmax)
            self.true_V = deque([0.0] * nmax, maxlen=nmax)
            self.true_alpha = deque([0.0] * nmax, maxlen=nmax)
            self.true_h = deque([0.0] * nmax, maxlen=nmax)
            self.true_heading = deque([0.0] * nmax, maxlen=nmax)
        
        print(f"[DASHBOARD] Pre-allocation complete (~{nmax * 18 * 8 / 1024:.1f} KB allocated)")

        if HAS_MPL:
            plt.ion()
            # Create figure with appropriate size based on whether sensors are included
            if include_sensors:
                # 5 rows x 2 columns for combined view
                self.fig, self.ax = plt.subplots(5, 2, figsize=(14, 12), constrained_layout=True)
                self.fig.suptitle('F-16 Simulation Dashboard - State & Sensors', fontsize=14, fontweight='bold')
            else:
                # 3 rows x 2 columns for state only
                self.fig, self.ax = plt.subplots(3, 2, figsize=(11, 8), constrained_layout=True)
                self.fig.suptitle('F-16 Simulation Dashboard', fontsize=14, fontweight='bold')
            
            self.lines = {}
            
            # Row 0: Alpha/Beta and Airspeed (State)
            self.lines['alpha'], = self.ax[0,0].plot([], [], 'b-', label='alpha [deg]', linewidth=1.5)
            self.lines['beta'],  = self.ax[0,0].plot([], [], 'g-', label='beta [deg]', linewidth=1.5)
            self.ax[0,0].set_title('Alpha/Beta'); self.ax[0,0].legend(loc='upper right'); self.ax[0,0].grid(True, alpha=0.3)
            self.lines['V'], = self.ax[0,1].plot([], [], 'b-', label='V [m/s]', linewidth=1.5)
            self.ax[0,1].set_title('Airspeed'); self.ax[0,1].grid(True, alpha=0.3)
            
            # Row 1: Body rates and Euler angles (State)
            self.lines['p'], = self.ax[1,0].plot([], [], 'b-', label='p [rad/s]', linewidth=1.5)
            self.lines['q'], = self.ax[1,0].plot([], [], 'g-', label='q [rad/s]', linewidth=1.5)
            self.lines['r'], = self.ax[1,0].plot([], [], 'r-', label='r [rad/s]', linewidth=1.5)
            self.ax[1,0].set_title('Body rates'); self.ax[1,0].legend(loc='upper right'); self.ax[1,0].grid(True, alpha=0.3)
            self.lines['roll'],  = self.ax[1,1].plot([], [], 'b-', label='roll φ [deg]', linewidth=1.5)
            self.lines['pitch'], = self.ax[1,1].plot([], [], 'g-', label='pitch θ [deg]', linewidth=1.5)
            self.lines['head'],  = self.ax[1,1].plot([], [], 'r-', label='heading ψ [deg]', linewidth=1.5)
            self.ax[1,1].set_title('Euler angles'); self.ax[1,1].legend(loc='upper right'); self.ax[1,1].grid(True, alpha=0.3)
            
            # Row 2: Altitude and Controls (State)
            self.lines['h'], = self.ax[2,0].plot([], [], 'b-', label='h [m]', linewidth=1.5)
            self.ax[2,0].set_title('Altitude'); self.ax[2,0].grid(True, alpha=0.3)
            self.lines['de'], = self.ax[2,1].plot([], [], 'b-', label='de [deg]', linewidth=1.5)
            self.lines['da'], = self.ax[2,1].plot([], [], 'g-', label='da [deg]', linewidth=1.5)
            self.lines['dr'], = self.ax[2,1].plot([], [], 'r-', label='dr [deg]', linewidth=1.5)
            self.ax[2,1].set_title('Controls'); self.ax[2,1].legend(loc='upper right'); self.ax[2,1].grid(True, alpha=0.3)
            
            # Sensor plots (if enabled)
            if include_sensors:
                # Row 3: IMU Sensors
                self.lines['accel_fx'], = self.ax[3,0].plot([], [], 'b-', label='fx [m/s²]', linewidth=1.5)
                self.lines['accel_fy'], = self.ax[3,0].plot([], [], 'g-', label='fy [m/s²]', linewidth=1.5)
                self.lines['accel_fz'], = self.ax[3,0].plot([], [], 'r-', label='fz [m/s²]', linewidth=1.5)
                self.ax[3,0].set_title('IMU Accelerometer'); self.ax[3,0].legend(loc='upper right'); self.ax[3,0].grid(True, alpha=0.3)
                
                self.lines['air_V'], = self.ax[3,1].plot([], [], 'b-', label='Airspeed [m/s]', linewidth=1.5)
                self.lines['true_V'], = self.ax[3,1].plot([], [], 'k--', label='True V [m/s]', linewidth=1.0, alpha=0.6)
                self.ax[3,1].set_title('Pitot Airspeed (vs True)'); self.ax[3,1].legend(loc='upper right'); self.ax[3,1].grid(True, alpha=0.3)
                
                # Row 4: GPS and Air Data
                self.lines['gps_alt'], = self.ax[4,0].plot([], [], 'b-', label='GPS Alt [m]', linewidth=1.5)
                self.lines['true_h'], = self.ax[4,0].plot([], [], 'k--', label='True Alt [m]', linewidth=1.0, alpha=0.6)
                self.ax[4,0].set_title('GPS Altitude (vs True)'); self.ax[4,0].legend(loc='upper right'); self.ax[4,0].grid(True, alpha=0.3)
                
                self.lines['air_alpha'], = self.ax[4,1].plot([], [], 'b-', label='α [deg]', linewidth=1.5)
                self.lines['true_alpha'], = self.ax[4,1].plot([], [], 'k--', label='True α [deg]', linewidth=1.0, alpha=0.6)
                self.ax[4,1].set_title('Air Data Alpha (vs True)'); self.ax[4,1].legend(loc='upper right'); self.ax[4,1].grid(True, alpha=0.3)

    def append(self, t, state, controls, sensor_measurements=None):
        """Append state data and optionally sensor data."""
        a_deg, b_deg, V_now = uvw_to_alphabeta(state["u"], state["v"], state["w"])
        self.t.append(t)
        self.alpha.append(a_deg); self.beta.append(b_deg); self.V.append(V_now)
        self.p.append(state['p']); self.q.append(state['q']); self.r.append(state['r'])
        self.roll.append(math.degrees(state['phi']))
        self.pitch.append(math.degrees(state['theta']))
        self.heading.append((math.degrees(state['psi']) % 360.0))
        self.h.append(state['h'])
        self.de.append(controls.get('de', 0.0)); self.da.append(controls.get('da', 0.0)); self.dr.append(controls.get('dr', 0.0))
        
        # Append sensor data if available and enabled
        if self.include_sensors and sensor_measurements is not None:
            imu_accel = sensor_measurements.get("imu_accel", {})
            self.accel_fx.append(imu_accel.get("fx", 0.0))
            self.accel_fy.append(imu_accel.get("fy", 0.0))
            self.accel_fz.append(imu_accel.get("fz", 0.0))
            
            gps = sensor_measurements.get("gps", {})
            self.gps_alt.append(gps.get("alt_m", 0.0))
            
            air = sensor_measurements.get("air_data", {})
            self.air_V.append(air.get("airspeed_mps", 0.0))
            self.air_alpha.append(air.get("alpha_deg", 0.0))
            
            # True values for comparison
            self.true_V.append(V_now)
            self.true_alpha.append(a_deg)
            self.true_h.append(state['h'])
            self.true_heading.append((math.degrees(state['psi']) % 360.0))

    def draw(self):
        """Update and redraw all plots (optimized for performance)."""
        if not HAS_MPL:
            return
        t = list(self.t)
        if not t:
            return
        
        def upd(key, y):
            if key in self.lines:
                self.lines[key].set_data(t, y)
        
        # Update state plots
        upd('alpha', list(self.alpha)); upd('beta', list(self.beta))
        upd('V', list(self.V))
        upd('p', list(self.p)); upd('q', list(self.q)); upd('r', list(self.r))
        upd('roll', list(self.roll)); upd('pitch', list(self.pitch)); upd('head', list(self.heading))
        upd('h', list(self.h))
        upd('de', list(self.de)); upd('da', list(self.da)); upd('dr', list(self.dr))
        
        # Update sensor plots if enabled
        if self.include_sensors:
            upd('accel_fx', list(self.accel_fx))
            upd('accel_fy', list(self.accel_fy))
            upd('accel_fz', list(self.accel_fz))
            upd('air_V', list(self.air_V))
            upd('gps_alt', list(self.gps_alt))
            upd('air_alpha', list(self.air_alpha))
            
            # True values for comparison
            if any(self.true_V):
                upd('true_V', list(self.true_V))
                upd('true_alpha', list(self.true_alpha))
                upd('true_h', list(self.true_h))

        # X-axis window: last window_sec seconds
        t_last = t[-1] if t else 0.0
        tmin = t_last - self.window_sec
        tmax = t_last
        if tmax - tmin < 1e-6:
            tmin = t_last - 1e-3
            tmax = t_last + 1e-3
        
        for ax in self.ax.ravel():
            ax.set_xlim(tmin, tmax)
        
        # Auto-scale y-axis (only every few updates to improve performance)
        # Only autoscale if we have enough data points
        if len(t) % 5 == 0:  # Every 5th update
            for a in self.ax.ravel():
                a.relim()
                a.autoscale_view()
        else:
            # Just update limits without full autoscale
            for a in self.ax.ravel():
                a.relim()
        
        # Optimized drawing - use draw_idle instead of draw for better performance
        self.fig.canvas.draw_idle()
        # Don't flush events every time - this is expensive
        # Only flush every few updates
        if len(t) % 3 == 0:
            try:
                self.fig.canvas.flush_events()
            except Exception:
                pass
        # Minimal pause - only when needed
        try:
            plt.pause(0.001)  # Minimal pause
        except Exception:
            pass

# --------------- Sensor Dashboard ------------------
class SensorDashboard:
    """Live dashboard for sensor measurements with noise visualization."""
    def __init__(self, window_sec, dt):
        self.window_sec = float(window_sec)
        nmax = int(window_sec / dt)
        self.t = deque(maxlen=nmax)
        
        # IMU data
        self.accel_fx = deque(maxlen=nmax); self.accel_fy = deque(maxlen=nmax); self.accel_fz = deque(maxlen=nmax)
        self.gyro_p = deque(maxlen=nmax); self.gyro_q = deque(maxlen=nmax); self.gyro_r = deque(maxlen=nmax)
        
        # GPS data
        self.gps_alt = deque(maxlen=nmax)
        self.gps_vn = deque(maxlen=nmax); self.gps_ve = deque(maxlen=nmax); self.gps_vd = deque(maxlen=nmax)
        
        # Air data
        self.air_V = deque(maxlen=nmax)
        self.air_alpha = deque(maxlen=nmax); self.air_beta = deque(maxlen=nmax)
        self.air_static_alt = deque(maxlen=nmax)
        
        # Magnetometer & Barometer
        self.mag_heading = deque(maxlen=nmax)
        self.baro_alt = deque(maxlen=nmax)
        
        # True values for comparison (when available)
        self.true_V = deque(maxlen=nmax)
        self.true_alpha = deque(maxlen=nmax)
        self.true_h = deque(maxlen=nmax)
        self.true_heading = deque(maxlen=nmax)
        
        if HAS_MPL:
            plt.ion()
            # Create a larger figure with 4 rows x 2 columns
            self.fig, self.ax = plt.subplots(4, 2, figsize=(14, 10), constrained_layout=True)
            self.fig.suptitle('Sensor Dashboard', fontsize=14, fontweight='bold')
            
            self.lines = {}
            
            # Row 0: IMU Accelerometer
            self.lines['accel_fx'], = self.ax[0,0].plot([], [], 'b-', label='fx [m/s²]', linewidth=1.5)
            self.lines['accel_fy'], = self.ax[0,0].plot([], [], 'g-', label='fy [m/s²]', linewidth=1.5)
            self.lines['accel_fz'], = self.ax[0,0].plot([], [], 'r-', label='fz [m/s²]', linewidth=1.5)
            self.ax[0,0].set_title('IMU Accelerometer (Specific Force)')
            self.ax[0,0].legend(loc='upper right'); self.ax[0,0].grid(True, alpha=0.3)
            
            # Row 0: IMU Gyroscope
            self.lines['gyro_p'], = self.ax[0,1].plot([], [], 'b-', label='p [rad/s]', linewidth=1.5)
            self.lines['gyro_q'], = self.ax[0,1].plot([], [], 'g-', label='q [rad/s]', linewidth=1.5)
            self.lines['gyro_r'], = self.ax[0,1].plot([], [], 'r-', label='r [rad/s]', linewidth=1.5)
            self.ax[0,1].set_title('IMU Gyroscope (Angular Rates)')
            self.ax[0,1].legend(loc='upper right'); self.ax[0,1].grid(True, alpha=0.3)
            
            # Row 1: GPS Position & Velocity
            self.lines['gps_alt'], = self.ax[1,0].plot([], [], 'b-', label='GPS Alt [m]', linewidth=1.5)
            self.lines['true_h'], = self.ax[1,0].plot([], [], 'k--', label='True Alt [m]', linewidth=1.0, alpha=0.6)
            self.ax[1,0].set_title('GPS Altitude')
            self.ax[1,0].legend(loc='upper right'); self.ax[1,0].grid(True, alpha=0.3)
            
            self.lines['gps_vn'], = self.ax[1,1].plot([], [], 'b-', label='vn [m/s]', linewidth=1.5)
            self.lines['gps_ve'], = self.ax[1,1].plot([], [], 'g-', label='ve [m/s]', linewidth=1.5)
            self.lines['gps_vd'], = self.ax[1,1].plot([], [], 'r-', label='vd [m/s]', linewidth=1.5)
            self.ax[1,1].set_title('GPS Velocity (NED)')
            self.ax[1,1].legend(loc='upper right'); self.ax[1,1].grid(True, alpha=0.3)
            
            # Row 2: Air Data
            self.lines['air_V'], = self.ax[2,0].plot([], [], 'b-', label='Airspeed [m/s]', linewidth=1.5)
            self.lines['true_V'], = self.ax[2,0].plot([], [], 'k--', label='True V [m/s]', linewidth=1.0, alpha=0.6)
            self.ax[2,0].set_title('Pitot Airspeed')
            self.ax[2,0].legend(loc='upper right'); self.ax[2,0].grid(True, alpha=0.3)
            
            self.lines['air_alpha'], = self.ax[2,1].plot([], [], 'b-', label='α [deg]', linewidth=1.5)
            self.lines['air_beta'], = self.ax[2,1].plot([], [], 'g-', label='β [deg]', linewidth=1.5)
            self.lines['true_alpha'], = self.ax[2,1].plot([], [], 'k--', label='True α [deg]', linewidth=1.0, alpha=0.6)
            self.ax[2,1].set_title('Air Data: Alpha & Beta')
            self.ax[2,1].legend(loc='upper right'); self.ax[2,1].grid(True, alpha=0.3)
            
            # Row 3: Magnetometer & Barometric Altimeter
            self.lines['mag_heading'], = self.ax[3,0].plot([], [], 'b-', label='Heading [deg]', linewidth=1.5)
            self.lines['true_heading'], = self.ax[3,0].plot([], [], 'k--', label='True Heading [deg]', linewidth=1.0, alpha=0.6)
            self.ax[3,0].set_title('Magnetometer Heading')
            self.ax[3,0].legend(loc='upper right'); self.ax[3,0].grid(True, alpha=0.3)
            
            self.lines['baro_alt'], = self.ax[3,1].plot([], [], 'b-', label='Baro Alt [m]', linewidth=1.5)
            self.lines['true_h_baro'], = self.ax[3,1].plot([], [], 'k--', label='True Alt [m]', linewidth=1.0, alpha=0.6)
            self.ax[3,1].set_title('Barometric Altimeter')
            self.ax[3,1].legend(loc='upper right'); self.ax[3,1].grid(True, alpha=0.3)
    
    def append(self, t, sensor_measurements, true_state=None):
        """Append sensor measurements and optionally true state for comparison."""
        if sensor_measurements is None:
            return
        
        self.t.append(t)
        
        # IMU
        imu_accel = sensor_measurements.get("imu_accel", {})
        self.accel_fx.append(imu_accel.get("fx", 0.0))
        self.accel_fy.append(imu_accel.get("fy", 0.0))
        self.accel_fz.append(imu_accel.get("fz", 0.0))
        
        imu_gyro = sensor_measurements.get("imu_gyro", {})
        self.gyro_p.append(imu_gyro.get("p", 0.0))
        self.gyro_q.append(imu_gyro.get("q", 0.0))
        self.gyro_r.append(imu_gyro.get("r", 0.0))
        
        # GPS
        gps = sensor_measurements.get("gps", {})
        self.gps_alt.append(gps.get("alt_m", 0.0))
        self.gps_vn.append(gps.get("vn_mps", 0.0))
        self.gps_ve.append(gps.get("ve_mps", 0.0))
        self.gps_vd.append(gps.get("vd_mps", 0.0))
        
        # Air Data
        air = sensor_measurements.get("air_data", {})
        self.air_V.append(air.get("airspeed_mps", 0.0))
        self.air_alpha.append(air.get("alpha_deg", 0.0))
        self.air_beta.append(air.get("beta_deg", 0.0))
        self.air_static_alt.append(air.get("static_alt_m", 0.0))
        
        # Magnetometer & Barometer
        mag = sensor_measurements.get("magnetometer", {})
        self.mag_heading.append(mag.get("heading_deg", 0.0))
        
        baro = sensor_measurements.get("baro_alt", {})
        self.baro_alt.append(baro.get("alt_m", 0.0))
        
        # True values for comparison (if provided)
        if true_state is not None:
            u = true_state.get("u", 0.0)
            v = true_state.get("v", 0.0)
            w = true_state.get("w", 0.0)
            alpha_true, beta_true, V_true = uvw_to_alphabeta(u, v, w)
            self.true_V.append(V_true)
            self.true_alpha.append(alpha_true)
            self.true_h.append(true_state.get("h", 0.0))
            self.true_heading.append((math.degrees(true_state.get("psi", 0.0)) % 360.0))
        else:
            # If no true state, duplicate sensor values (no comparison line)
            self.true_V.append(0.0)
            self.true_alpha.append(0.0)
            self.true_h.append(0.0)
            self.true_heading.append(0.0)
    
    def draw(self):
        """Update and redraw all plots."""
        if not HAS_MPL:
            return
        t = list(self.t)
        if not t:
            return
        
        def upd(key, y):
            if key in self.lines:
                self.lines[key].set_data(t, y)
        
        # Update all lines
        upd('accel_fx', list(self.accel_fx))
        upd('accel_fy', list(self.accel_fy))
        upd('accel_fz', list(self.accel_fz))
        
        upd('gyro_p', list(self.gyro_p))
        upd('gyro_q', list(self.gyro_q))
        upd('gyro_r', list(self.gyro_r))
        
        upd('gps_alt', list(self.gps_alt))
        upd('gps_vn', list(self.gps_vn))
        upd('gps_ve', list(self.gps_ve))
        upd('gps_vd', list(self.gps_vd))
        
        upd('air_V', list(self.air_V))
        upd('air_alpha', list(self.air_alpha))
        upd('air_beta', list(self.air_beta))
        
        upd('mag_heading', list(self.mag_heading))
        upd('baro_alt', list(self.baro_alt))
        
        # True values (only show if non-zero)
        if any(self.true_V):
            upd('true_V', list(self.true_V))
            upd('true_alpha', list(self.true_alpha))
            upd('true_h', list(self.true_h))
            upd('true_heading', list(self.true_heading))
            upd('true_h_baro', list(self.true_h))  # Same true altitude for baro
        
        # X-axis window: last window_sec seconds
        t_last = t[-1] if t else 0.0
        tmin = t_last - self.window_sec
        tmax = t_last
        if tmax - tmin < 1e-6:
            tmin = t_last - 1e-3
            tmax = t_last + 1e-3
        
        for ax in self.ax.ravel():
            ax.set_xlim(tmin, tmax)
        
        # Auto-scale y-axis for all plots
        for a in self.ax.ravel():
            a.relim()
            a.autoscale_view()
        
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
print(f"[SIM] Duration: {T_FINAL:.1f} s | Time step: {DT} s | Total steps: {N_STEPS} | Maneuver: {MANEUVER_TYPE if ENABLE_MANEUVER else 'none'}")
print(f"[SIM] Real-time: {'ON' if REAL_TIME else 'OFF'} | Time scale: {TIME_SCALE}x")
print(f"[SIM] Update rates: Dashboard={DASH_RATE} Hz | FlightGear={FG_UPDATE_RATE} Hz | Sensors={SENSOR_DASH_RATE} Hz")
print(f"[SIM] Resource usage: ~{int(PLOT_WINDOW_SEC/DT)*18*8/1024} KB RAM (dashboard) | ~{int(100*FG_UPDATE_RATE*50/8/1024)} Kbps network")

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

# Pre-allocate system resources before starting simulation
print("[INIT] Pre-allocating system resources...")
preallocated_arrays = preallocate_resources()

# Real-time synchronization
if REAL_TIME:
    loop_start_time = time.time()
    print("[INFO] Running in REAL-TIME mode - simulation will match wall-clock time")
else:
    print("[INFO] Running in FAST mode - no real-time synchronization")

# Initialize dashboard - combine if both enabled and COMBINE_DASHBOARDS is True
if COMBINE_DASHBOARDS and ENABLE_DASHBOARD and ENABLE_SENSOR_DASHBOARD:
    dash = CombinedDashboard(PLOT_WINDOW_SEC, DT, include_sensors=True)
    sensor_dash = None  # Combined into main dashboard
    print("[DASHBOARD] Combined dashboard enabled (State + Sensors)")
elif ENABLE_DASHBOARD:
    dash = CombinedDashboard(PLOT_WINDOW_SEC, DT, include_sensors=False)
    sensor_dash = SensorDashboard(PLOT_WINDOW_SEC, DT) if ENABLE_SENSOR_DASHBOARD else None
    print("[DASHBOARD] State dashboard enabled" + (" + separate sensor dashboard" if sensor_dash else ""))
else:
    dash = None
    sensor_dash = SensorDashboard(PLOT_WINDOW_SEC, DT) if ENABLE_SENSOR_DASHBOARD else None

# Initialize web dashboard (recommended - much lower CPU usage)
web_dash = None
if ENABLE_WEB_DASHBOARD:
    try:
        print(f"[WEB] Initializing web dashboard on port {WEB_DASHBOARD_PORT}...")
        web_dash = WebDashboard(port=WEB_DASHBOARD_PORT)
        web_dash.start_server(run_in_thread=True)
        print(f"[WEB] Web dashboard initialization complete")
        print(f"[WEB] If connection fails, check console for errors above")
    except ImportError as e:
        print(f"[WEB] ❌ Failed: Flask-SocketIO not installed")
        print(f"[WEB] Install with: pip install flask flask-socketio")
        web_dash = None
    except Exception as e:
        print(f"[WEB] ❌ Failed to start web dashboard: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        print(f"[WEB] Continuing without web dashboard...")
        web_dash = None
else:
    if HAS_WEB_DASHBOARD:
        print("[WEB] Web dashboard disabled (set ENABLE_WEB_DASHBOARD = True to enable)")

# --------------- Initialize Sensor Suite ------------------
sensors = None
if ENABLE_SENSORS:
    sensors = SensorSuite(
        enable_noise=SENSOR_NOISE,
        enable_bias=SENSOR_BIAS,
        seed=42  # Fixed seed for reproducibility
    )
    print(f"[SENSORS] Sensor suite initialized (noise={'ON' if SENSOR_NOISE else 'OFF'}, bias={'ON' if SENSOR_BIAS else 'OFF'})")

# --------------- Initialize Sensor Fusion (EKF) ------------------
ekf = None
if ENABLE_SENSOR_FUSION and HAS_EKF and ENABLE_SENSORS:
    ekf = EKFSensorFusion(
        lat0_deg=LAT0_DEG,
        lon0_deg=LON0_DEG,
        dt=DT
    )
    # Will initialize after first state update
    print(f"[EKF] Sensor fusion EKF initialized (will initialize with first state)")
elif ENABLE_SENSOR_FUSION and not HAS_EKF:
    print(f"[EKF] Sensor fusion requested but EKF module not available")
elif ENABLE_SENSOR_FUSION and not ENABLE_SENSORS:
    print(f"[EKF] Sensor fusion requires sensors to be enabled")

# --------------- Initialize EKF Dashboard ------------------
ekf_dash = None
if ENABLE_EKF_DASHBOARD and ENABLE_SENSOR_FUSION:
    try:
        print(f"[EKF-DASH] Initializing EKF dashboard on port {EKF_DASHBOARD_PORT}...")
        ekf_dash = EKFDashboard(port=EKF_DASHBOARD_PORT)
        ekf_dash.start_server(run_in_thread=True)
        print(f"[EKF-DASH] EKF dashboard initialization complete")
    except ImportError as e:
        print(f"[EKF-DASH] ❌ Failed: Flask-SocketIO not installed")
        print(f"[EKF-DASH] Install with: pip install flask flask-socketio")
        ekf_dash = None
    except Exception as e:
        print(f"[EKF-DASH] ❌ Failed to start EKF dashboard: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        print(f"[EKF-DASH] Continuing without EKF dashboard...")
        ekf_dash = None
else:
    if HAS_EKF_DASHBOARD:
        print("[EKF-DASH] EKF dashboard disabled (set ENABLE_EKF_DASHBOARD = True to enable)")

# Main simulation loop
for step in range(N_STEPS):
    # Real-time synchronization (only check periodically to reduce overhead)
    if REAL_TIME and step % RT_SYNC_INTERVAL == 0:
        expected_time = step * DT / TIME_SCALE
        actual_time = time.time() - loop_start_time
        sleep_time = expected_time - actual_time
        if sleep_time > 0.002:  # Only sleep if significant delay needed (increased threshold)
            time.sleep(min(sleep_time, 0.05))  # Reduced cap to prevent large jumps
    
    # Periodic garbage collection (every 1000 steps to prevent memory buildup)
    if step % 1000 == 0:
        gc.collect()  # Force garbage collection periodically
    
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

    # 3.4) Update sensors (after state integration)
    # Update sensors at dashboard rate to ensure data is available when needed
    sensor_measurements = None
    if sensors is not None and (step % DASH_UPDATE_INTERVAL == 0):
        # Get forces for accelerometer (simplified - skip expensive aero calculation for performance)
        forces_dict = {
            "Fx": 0.0, "Fy": 0.0, "Fz": 0.0  # Simplified for performance
        }
        # Only compute forces if really needed (skip for performance)
        # Sensor will use approximate acceleration from kinematics
        
        sensor_measurements = sensors.update(
            t, state, N, E, D, LAT0_DEG, LON0_DEG, DT, forces=forces_dict
        )
        
        # Log sensor outputs periodically
        if step % SENSOR_LOG_INTERVAL == 0:
            imu = sensor_measurements["imu_accel"]
            gps = sensor_measurements["gps"]
            air = sensor_measurements["air_data"]
            print(f"[SENSORS] t={t:.2f}s | "
                  f"IMU: fx={imu['fx']:.2f} fy={imu['fy']:.2f} fz={imu['fz']:.2f} m/s² | "
                  f"GPS: alt={gps['alt_m']:.1f}m | "
                  f"Air: V={air['airspeed_mps']:.1f}m/s alpha={air['alpha_deg']:.2f}°")
    
    # 3.5) Update sensor fusion (EKF) - runs at higher rate
    estimated_state = None
    if ekf is not None and sensor_measurements is not None:
        # Initialize EKF on first update
        if not ekf.initialized:
            ekf.initialize(state, N, E, D)
            print(f"[EKF] EKF initialized with initial state")
        
        # Update EKF at fusion rate
        if step % SENSOR_FUSION_INTERVAL == 0:
            try:
                # Predict step (using IMU)
                ekf.predict()
                
                # Update with all measurements
                if "imu_accel" in sensor_measurements and "imu_gyro" in sensor_measurements:
                    ekf.update_imu(sensor_measurements["imu_accel"], sensor_measurements["imu_gyro"])
                
                if "gps" in sensor_measurements and sensor_measurements["gps"] is not None:
                    ekf.update_gps(sensor_measurements["gps"])
                
                if "air_data" in sensor_measurements:
                    ekf.update_air_data(sensor_measurements["air_data"])
                
                if "magnetometer" in sensor_measurements:
                    ekf.update_magnetometer(sensor_measurements["magnetometer"])
                
                if "baro_alt" in sensor_measurements:
                    ekf.update_baro(sensor_measurements["baro_alt"])
                
                # Get estimated state
                estimated_state = ekf.get_estimated_state()
                
                # Log EKF state periodically
                if step % SENSOR_LOG_INTERVAL == 0:
                    est = estimated_state
                    true_h = state["h"]
                    est_h = est["h"]
                    error_h = abs(true_h - est_h)
                    true_V = math.sqrt(state["u"]**2 + state["v"]**2 + state["w"]**2)
                    est_V = est["V"]
                    error_V = abs(true_V - est_V)
                    print(f"[EKF] t={t:.2f}s | Est h={est_h:.1f}m (err={error_h:.2f}m) | Est V={est_V:.1f}m/s (err={error_V:.2f}m/s)")
                
                # Send to EKF dashboard (only if EKF is working)
                if ekf_dash is not None and estimated_state is not None:
                    # Calculate errors
                    true_h = state["h"]
                    est_h = estimated_state["h"]
                    true_V = math.sqrt(state["u"]**2 + state["v"]**2 + state["w"]**2)
                    est_V = estimated_state["V"]
                    true_roll = math.degrees(state["phi"])
                    est_roll = math.degrees(estimated_state["phi"])
                    true_pitch = math.degrees(state["theta"])
                    est_pitch = math.degrees(estimated_state["theta"])
                    
                    # Position error (3D)
                    pos_error = abs(true_h - est_h)  # Simplified to altitude error
                    
                    # Velocity error
                    vel_error = abs(true_V - est_V)
                    
                    # Attitude error (combined roll/pitch)
                    roll_error = abs(true_roll - est_roll)
                    pitch_error = abs(true_pitch - est_pitch)
                    att_error = math.sqrt(roll_error**2 + pitch_error**2)
                    
                    # EKF parameters (covariance diagonal)
                    P_diag = np.diag(ekf.P)
                    
                    ekf_data = {
                        "t": t,
                        "true_state": {
                            "h": true_h,
                            "V": true_V,
                            "roll": true_roll,
                            "pitch": true_pitch,
                            "heading": math.degrees(state["psi"])
                        },
                        "estimated_state": {
                            "h": est_h,
                            "V": est_V,
                            "roll": est_roll,
                            "pitch": est_pitch,
                            "heading": math.degrees(estimated_state["psi"]),
                            "ba": estimated_state["ba"].tolist() if isinstance(estimated_state["ba"], np.ndarray) else estimated_state["ba"],
                            "bg": estimated_state["bg"].tolist() if isinstance(estimated_state["bg"], np.ndarray) else estimated_state["bg"]
                        },
                        "errors": {
                            "position_error": pos_error,
                            "velocity_error": vel_error,
                            "attitude_error": att_error
                        },
                        "ekf_params": {
                            "P_position": float(P_diag[2]),  # D (altitude) covariance
                            "P_velocity": float(np.mean(P_diag[3:6])),  # Average velocity covariance
                            "P_attitude": float(np.mean(P_diag[6:9])),  # Average attitude covariance
                            "P_accel_bias": float(np.mean(P_diag[9:12])),  # Average accel bias covariance
                            "P_gyro_bias": float(np.mean(P_diag[12:15]))  # Average gyro bias covariance
                        },
                        "stats": ekf_dash.get_statistics() if ekf_dash else {}
                    }
                    ekf_dash.send_data(ekf_data)
            except Exception as e:
                if step % SENSOR_LOG_INTERVAL == 0:
                    print(f"[EKF] Error in update: {e}")
    
    # Update separate sensor dashboard (only if not combined)
    if sensor_dash is not None and (step % SENSOR_DASH_UPDATE_INTERVAL == 0) and sensor_measurements is not None:
        sensor_dash.append(t, sensor_measurements, true_state=state)
        sensor_dash.draw()

    # 3) Send to FlightGear (optional) - throttled to FG_UPDATE_RATE
    if fg is not None and (step % FG_SEND_INTERVAL == 0):
        try:
            # Optimized geodetic conversion (inline, no function call overhead)
            lat_deg = LAT0_DEG + (N / 111320.0)
            lon_cos = math.cos(math.radians(LAT0_DEG))
            lon_deg = LON0_DEG + (E / (111320.0 * lon_cos))
            alt_m = -D
            
            # Pre-calculate degrees to avoid repeated conversions
            roll_deg  = math.degrees(state["phi"])
            pitch_deg = math.degrees(state["theta"])
            heading   = (math.degrees(state["psi"]) % 360.0)
            
            # Clamp values (optimized order)
            roll_deg = max(-180.0, min(180.0, roll_deg))
            pitch_deg = max(-90.0, min(90.0, pitch_deg))
            heading = max(0.0, min(360.0, heading))
            alt_m = max(-1000.0, min(50000.0, alt_m))
            
            # Quick finite check (skip if any is NaN/Inf)
            if (math.isfinite(roll_deg) and math.isfinite(pitch_deg) and 
                math.isfinite(heading) and math.isfinite(alt_m) and
                math.isfinite(lat_deg) and math.isfinite(lon_deg)):
                fg.send_pose(lat_deg, lon_deg, alt_m, roll_deg, pitch_deg, heading)
        except Exception as e:
            # Only log errors occasionally
            if step % int(10.0/DT) == 0:
                print(f"[UDP Error] {str(e)}")

    # 3.5) Dashboard update (throttled)
    if dash is not None and (step % DASH_UPDATE_INTERVAL == 0):
        # Pass sensor measurements if combined dashboard (ensure it's not None)
        sensor_data = sensor_measurements if (COMBINE_DASHBOARDS and sensor_measurements is not None) else None
        dash.append(t, state, controls, sensor_measurements=sensor_data)
        dash.draw()
    
    # 3.6) Web dashboard update (throttled - much lower CPU than matplotlib)
    if web_dash is not None and (step % WEB_DASHBOARD_INTERVAL == 0):
        # Prepare data for web dashboard
        alpha_deg, beta_deg, V_now = uvw_to_alphabeta(state["u"], state["v"], state["w"])
        web_data = {
            "t": t,
            "state": {
                "V": V_now,
                "h": state["h"],
                "alpha": alpha_deg,
                "beta": beta_deg,
                "p": state["p"],
                "q": state["q"],
                "r": state["r"],
                "roll": math.degrees(state["phi"]),
                "pitch": math.degrees(state["theta"]),
                "heading": (math.degrees(state["psi"]) % 360.0)
            }
        }
        
        # Add sensor data if available
        if sensor_measurements is not None:
            web_data["sensors"] = {
                "imu_accel": sensor_measurements.get("imu_accel", {}),
                "gps": sensor_measurements.get("gps", {}),
                "air_data": sensor_measurements.get("air_data", {})
            }
        
        # Send to web dashboard (non-blocking)
        web_dash.send_data(web_data)

    # 4) Advance sim time
    t += DT


print("\nDone. Final state:")
alpha_deg, beta_deg, V_now = uvw_to_alphabeta(state["u"], state["v"], state["w"])
print(f"V={V_now:.2f} m/s, alpha={alpha_deg:.2f} deg, theta={math.degrees(state['theta']):.2f} deg, h={state['h']:.1f} m")
print(f"N={N:.1f} m, E={E:.1f} m, D={D:.1f} m")
