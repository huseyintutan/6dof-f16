# F-16 FlightGear Integration and Python Simulation Project

A real-time flight dynamics simulation system that couples a Python-based **F-16 six-degree-of-freedom (6-DoF) flight dynamics model** with the **FlightGear flight simulator** for interactive 3D visualization. This project provides a modular, physically-consistent framework suitable for flight dynamics research, control system development, and educational purposes.

---

## Project Overview

The main objectives of this project are:

* **Develop a simplified but physically consistent F-16 model** in Python with 6-DoF rigid body dynamics
* **Trim the aircraft** for steady-level flight conditions
* **Integrate with FlightGear** via UDP communication and generic protocol XML
* **Automate the workflow** through batch scripts for seamless execution

---

## Flight Dynamics Equations

The 6-DOF rigid body dynamics are expressed in body-axis coordinates.

### State Vector:

$$\mathbf{x} = [u, v, w, p, q, r, \phi, \theta, \psi, h]^T$$

Where:
* $(u, v, w)$: body-axis velocities [m/s]
* $(p, q, r)$: angular rates (roll, pitch, yaw) [rad/s]
* $(\phi, \theta, \psi)$: Euler angles [rad]
* $h$: altitude [m]

### Translational Dynamics:

$$\begin{bmatrix} \dot{u} \\ \dot{v} \\ \dot{w} \end{bmatrix} = \begin{bmatrix} r v - q w \\ p w - r u \\ q u - p v \end{bmatrix} + \frac{1}{m}\begin{bmatrix} X \\ Y \\ Z \end{bmatrix} + g\begin{bmatrix} -\sin\theta \\ \sin\phi\cos\theta \\ \cos\phi\cos\theta \end{bmatrix}$$

**Component form:**
- $\dot{u} = rv - qw + X/m - g\sin\theta$
- $\dot{v} = pw - ru + Y/m + g\sin\phi\cos\theta$
- $\dot{w} = qu - pv + Z/m + g\cos\phi\cos\theta$

### Rotational Dynamics:

$$\begin{bmatrix} \dot{p} \\ \dot{q} \\ \dot{r} \end{bmatrix} = I^{-1} \left( \begin{bmatrix} L \\ M \\ N \end{bmatrix} - \begin{bmatrix} p \\ q \\ r \end{bmatrix} \times \left(I \begin{bmatrix} p \\ q \\ r \end{bmatrix}\right) \right)$$

### Kinematic Equations (Euler and Quaternion):

$$\begin{aligned}
\dot{\phi} &= p + \tan\theta (q\sin\phi + r\cos\phi) \\
\dot{\theta} &= q\cos\phi - r\sin\phi \\
\dot{\psi} &= \frac{q\sin\phi + r\cos\phi}{\cos\theta}
\end{aligned}$$

Implementation note: the simulator propagates attitude with a unit quaternion using RK4 and derives Euler angles for logging/FlightGear output. This avoids gimbal singularities.

### Altitude Rate:

$$\dot{h} = -u\sin\theta - v\sin\phi\cos\theta - w\cos\phi\cos\theta$$

---

## Aerodynamic Forces and Moments

Each aerodynamic coefficient (e.g., $C_L$, $C_D$, $C_m$) is interpolated from tabulated data as a function of:
* Mach number
* Angle of attack ($\alpha$)
* Sideslip angle ($\beta$)
* Control deflections (elevator $\delta_e$, aileron $\delta_a$, rudder $\delta_r$)

### Forces:

$$\begin{bmatrix} X \\ Y \\ Z \end{bmatrix} = \frac{1}{2}\rho V^2 S \begin{bmatrix} -C_D \\ C_Y \\ -C_L \end{bmatrix}$$

**Component form:**
- $X = -\frac{1}{2}\rho V^2 S C_D$
- $Y = \frac{1}{2}\rho V^2 S C_Y$
- $Z = -\frac{1}{2}\rho V^2 S C_L$

### Moments:

$$\begin{bmatrix} L \\ M \\ N \end{bmatrix} = \frac{1}{2}\rho V^2 S \begin{bmatrix} b C_l \\ c_{\bar{c}} C_m \\ b C_n \end{bmatrix}$$

**Component form:**
- $L = \frac{1}{2}\rho V^2 S b C_l$ (rolling moment)
- $M = \frac{1}{2}\rho V^2 S c_{\bar{c}} C_m$ (pitching moment)
- $N = \frac{1}{2}\rho V^2 S b C_n$ (yawing moment)

Where:
### Lateral-Directional Minimal Model and Dynamic Derivatives

When the aero database lacks lateral-directional entries, a minimal model is used to introduce stability/controls effects:

- $C_Y = C_{Y_\beta}\,\beta$
- $C_l = C_{l_\beta}\,\beta + C_{l_p}\,\hat p + C_{l_{\delta_a}}\,\delta_a + C_{l_r}\,\hat r$
- $C_n = C_{n_\beta}\,\beta + C_{n_r}\,\hat r + C_{n_{\delta_r}}\,\delta_r + C_{n_p}\,\hat p$

Dynamic derivatives included: $C_{m_q}\,\hat q$, $C_{L_q}\,\hat q$ with $\hat p = pb/(2V)$, $\hat q = qc/(2V)$, $\hat r = rb/(2V)$.
* $\rho$: air density [kg/m³]
* $V$: airspeed [m/s]
* $S$: wing reference area [m²]
* $b$: wingspan [m]
* $c_{\bar{c}}$: mean aerodynamic chord [m]

---

## Trimming for Level Flight

The trim function `trim_level_flight(V0, h0)` solves for angle of attack $\alpha$, pitch $\theta$, elevator $\delta_e$, and thrust $T$ such that steady-level flight holds:

$$\dot{w} = \dot{q} = \dot{h} = 0 \quad \Rightarrow \quad L = W, \quad T = D$$

### Algorithm:

1. **Initial guess** for $\alpha$, $\theta$, $\delta_e$, $T$
2. **Propagate dynamics** → compute residuals ($\dot u$, $\dot w$, $\dot q$, $\dot h$)
3. **Newton iteration** with finite-difference Jacobian
4. **Return** trimmed state and control inputs

---
## Quick Start Guide

### Prerequisites

* Python 3.7+ with NumPy
* FlightGear 2024.1 (or compatible version)
* F-16 aircraft model (`f16-block-52` from fgaddon)
* Windows (for batch files; Linux/Mac users can adapt scripts)

### Installation

1. **Clone or download** this repository

2. **Install Python dependencies**:
   ```bash
   pip install numpy
   ```

3. **Install web dashboard dependencies (optional, recommended)**:
   ```bash
   scripts\install_web_dashboard.bat
   # OR
   pip install flask flask-socketio
   ```

4. **Place `myproto.xml` in FlightGear protocols directory**:
   ```
   C:\Users\<YourUser>\FlightGear\fgdata\Protocols\myproto.xml
   ```

5. **Update paths in batch files**:
   - Edit `scripts/start_flightgear.bat` with your FlightGear installation path
   - The `scripts/run_sim.bat` automatically detects the project root directory

### Running the Simulation

#### Option 1: Manual Launch (Recommended)

1. **Start FlightGear first**:
   ```bash
   scripts\start_flightgear.bat
   ```
   Wait until FlightGear fully loads.

2. **Run the simulation**:
   ```bash
   scripts\run_sim.bat
   ```

#### Option 2: Direct Python Execution

```bash
python main.py
```

### Configuration

Edit `main.py` to adjust simulation parameters:

```python
DT       = 0.01         # time step [s]
T_FINAL  = 10.0         # total time [s]
LAT0_DEG = 41.015137    # reference latitude (Istanbul)
LON0_DEG = 28.979530    # reference longitude
SEND_TO_FG = True       # enable FlightGear UDP output
ENABLE_MANEUVER = True  # internal maneuver inputs (turn/roll/climb)
MANEUVER_TYPE = "turn"   # "climb"|"turn"|"roll"|"none"
ENABLE_WEB_DASHBOARD = True  # web-based dashboard (recommended, low CPU)
WEB_DASHBOARD_PORT = 5000    # web dashboard port
ENABLE_DASHBOARD = False     # matplotlib dashboard (legacy, high CPU)
DASH_RATE = 10.0             # Hz (matplotlib only)
PLOT_WINDOW_SEC = 20.0       # seconds (matplotlib only)
ENABLE_SENSORS = True        # sensor modeling
SENSOR_NOISE = True          # sensor noise
SENSOR_BIAS = True           # sensor bias
```

Edit `f16_constants.py` to modify aircraft trim conditions:

```python
V_TRIM = 150.0     # trim speed [m/s]
H_TRIM = 3048.0    # trim altitude [m]
```

---

## Coordinate Systems and Conventions

### NED Frame (North-East-Down)
* **+N**: North (forward in local horizontal plane)
* **+E**: East (right in local horizontal plane)
* **+D**: Down (positive downward)

### Body Axes
* **+x**: Forward (aircraft nose)
* **+y**: Right wing
* **+z**: Down (into aircraft)

### Euler Angles (3-2-1 Sequence)
* **ψ (Yaw)**: Rotation about z-axis
* **θ (Pitch)**: Rotation about y-axis
* **φ (Roll)**: Rotation about x-axis

---

## Live Dashboards

The simulation supports two types of live dashboards:

### Web Dashboard (Recommended)

A lightweight, web-based dashboard with much lower CPU usage (~5-15% vs 40-60% for matplotlib). Access it via your web browser at `http://127.0.0.1:5000` after starting the simulation.

**Features:**
- Real-time state visualization (airspeed, altitude, attitude, angular rates)
- Sensor data display (IMU, GPS, Air Data)
- Interactive charts using Chart.js
- WebSocket-based real-time updates
- Cross-platform compatible (works on any device with a browser)

**Setup:**
1. Install dependencies:
   ```bash
   scripts\install_web_dashboard.bat
   # OR
   pip install flask flask-socketio
   ```

2. Enable in `main.py`:
   ```python
   ENABLE_WEB_DASHBOARD = True
   WEB_DASHBOARD_PORT = 5000
   ```

3. Run simulation:
   ```bash
   scripts\run_sim.bat
   ```

4. Open dashboard:
   ```bash
   scripts\open_web_dashboard.bat
   # OR manually open: http://127.0.0.1:5000
   ```

**Check connection:**
```bash
scripts\check_web_dashboard.bat
```

### Matplotlib Dashboard (Legacy)

Enable `ENABLE_DASHBOARD = True` to open a live multi-panel plot (alpha/beta, airspeed, p–q–r, Euler angles, altitude, controls). Update rate is set by `DASH_RATE` and the visible time window by `PLOT_WINDOW_SEC`.

**Note:** The matplotlib dashboard has higher CPU usage and is disabled by default. Use the web dashboard for better performance.

---

## Sensor Modeling

The simulation includes a comprehensive sensor suite with realistic noise and bias modeling:

### Available Sensors

1. **IMU (Inertial Measurement Unit)**
   - **Accelerometer**: 3-axis specific force (fx, fy, fz) in body frame [m/s²]
   - **Gyroscope**: 3-axis angular rates (p, q, r) [rad/s]
   - Configurable noise and bias

2. **GPS (Global Positioning System)**
   - Position: latitude, longitude, altitude [deg, m]
   - Velocity: NED velocity components [m/s]
   - Update rate: 10 Hz (configurable)
   - Delay: 0.1 s (configurable)

3. **Air Data Sensors**
   - **Pitot-static**: Airspeed [m/s] and static pressure altitude [m]
   - **Angle of Attack (α)**: [degrees]
   - **Sideslip (β)**: [degrees]

4. **Magnetometer**
   - Heading measurement [degrees]

5. **Barometric Altimeter**
   - Altitude from static pressure [m]

### Usage

Sensors are automatically enabled in `main.py` when `ENABLE_SENSORS = True`. Configure sensor behavior:

```python
ENABLE_SENSORS = True          # Enable sensor modeling
SENSOR_NOISE = True            # Enable sensor noise
SENSOR_BIAS = True             # Enable sensor bias
SENSOR_LOG_RATE = 1.0          # Hz - Rate to print sensor outputs
```

Sensor measurements are available in the simulation loop:

```python
sensor_measurements = sensors.update(t, state, N, E, D, lat0, lon0, dt, forces)
# Access: sensor_measurements["imu_accel"], sensor_measurements["gps"], etc.
```

See `examples/sensor_example.py` for standalone sensor usage.

---

## Project Structure

```
sim_cloud/
├── main.py                 # Main simulation loop
├── src/
│   ├── f16_dynamics.py     # 6-DoF dynamics equations
│   ├── f16_forces.py       # Aerodynamic forces and moments
│   ├── f16_aero_loader.py  # Aerodynamic data loading
│   ├── f16_sensors.py      # Sensor modeling (IMU, GPS, Air Data, etc.)
│   ├── f16_kinematics.py   # Kinematic equations
│   ├── f16_atmosphere.py   # Atmospheric models
│   ├── gravity_model.py    # Gravity calculations
│   ├── earth_model.py      # Geodetic coordinate conversions
│   ├── f16_constants.py    # Aircraft constants and trim settings
│   └── web_dashboard.py    # Web dashboard server (Flask-SocketIO)
├── scripts/
│   ├── run_sim.bat         # Run Python simulation
│   ├── start_flightgear.bat # Start FlightGear
│   ├── install_web_dashboard.bat # Install web dashboard dependencies
│   ├── open_web_dashboard.bat    # Open web dashboard in browser
│   └── check_web_dashboard.bat   # Check web dashboard connection
├── templates/
│   └── dashboard.html      # Web dashboard HTML template
├── data/
│   └── F16_database.json   # Aerodynamic lookup tables
└── requirements.txt        # Python dependencies
```

## Future Improvements

* **Wind-axis equations** and advanced atmosphere models (winds, turbulence)
* **EKF sensor fusion** for state estimation from noisy sensor measurements
* **Bidirectional UDP link** (Python ←→ FlightGear for controls)
* **Autopilot implementation** (altitude hold, heading hold, etc.) using sensor feedback
* **Reinforcement Learning** integration for flight control
* **Data logging** (CSV/JSON export for post-flight analysis)
* **Multi-aircraft support** for formation flight simulations
* **Enhanced web dashboard** with 3D visualization and more sensor plots

---

## Technical References

* **Aircraft Dynamics**: Stevens, B. L., & Lewis, F. L. (2003). *Aircraft Control and Simulation*
* **FlightGear Documentation**: [https://wiki.flightgear.org/](https://wiki.flightgear.org/)
* **WGS-84**: NIMA (2000). *Department of Defense World Geodetic System 1984*
* **ISA Atmosphere**: ISO 2533:1975
