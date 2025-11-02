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

## Project Structure

```
sim_cloud/
├── src/                      # Python source modules
│   ├── __init__.py
│   ├── f16_constants.py      # Aircraft physical constants
│   ├── f16_dynamics.py       # 6-DoF dynamics & trim solver
│   ├── f16_forces.py         # Aerodynamic forces & moments
│   ├── f16_aero_loader.py    # Aerodynamic database loader
│   ├── f16_kinematics.py     # Position integration (NED)
│   ├── f16_atmosphere.py     # ISA atmosphere model
│   ├── gravity_model.py      # WGS-84 gravity model
│   └── earth_model.py        # WGS-84 Earth utilities
├── data/                     # Data files
│   └── F16_database.json     # Aerodynamic coefficients
├── scripts/                  # Batch scripts
│   ├── run_sim.bat          # Run simulation
│   └── start_flightgear.bat # Launch FlightGear
├── tests/                    # Test scripts
│   └── fg_send_test.py      # FlightGear UDP test
├── main.py                   # Entry point
└── README.md                 # This file
```

## Core Files and Their Roles

| File                   | Location    | Purpose                                                                                             |
| ---------------------- | ----------- | --------------------------------------------------------------------------------------------------- |
| `f16_dynamics.py`      | `src/`      | Implements aircraft dynamics equations, trim solver, and state propagation using RK4 integration  |
| `f16_forces.py`        | `src/`      | Computes aerodynamic forces & moments based on aerodynamic database lookups                         |
| `f16_aero_loader.py`   | `src/`      | Loads aerodynamic coefficient tables from JSON files (AoA, Mach, control surface grids)            |
| `f16_constants.py`     | `src/`      | Defines aircraft physical constants (mass, inertia, geometry, trim conditions)                     |
| `f16_kinematics.py`    | `src/`      | Handles position integration and coordinate transformations (NED frame)                            |
| `f16_atmosphere.py`    | `src/`      | Implements International Standard Atmosphere (ISA) model                                          |
| `gravity_model.py`     | `src/`      | WGS-84 gravity model with height correction                                                        |
| `earth_model.py`       | `src/`      | WGS-84 Earth model utilities for geodetic/ECEF conversions                                         |
| `main.py`              | root        | Entry point for simulation, handles initialization, trimming, and main simulation loop            |
| `F16_database.json`     | `data/`     | Aerodynamic coefficient tables                                                                    |
| `start_flightgear.bat` | `scripts/`  | Launches FlightGear in external FDM mode with UDP generic protocol                                |
| `run_sim.bat`          | `scripts/`  | Runs the Python simulation and sends data via UDP                                                  |

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

### Kinematic Equations:

$$\begin{aligned}
\dot{\phi} &= p + \tan\theta (q\sin\phi + r\cos\phi) \\
\dot{\theta} &= q\cos\phi - r\sin\phi \\
\dot{\psi} &= \frac{q\sin\phi + r\cos\phi}{\cos\theta}
\end{aligned}$$

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
* $\rho$: air density [kg/m³]
* $V$: airspeed [m/s]
* $S$: wing reference area [m²]
* $b$: wingspan [m]
* $c_{\bar{c}}$: mean aerodynamic chord [m]

---

## Trimming for Level Flight

The trim function `trim_level_flight(V0, h0)` numerically solves for the pitch angle $\theta$, elevator deflection $\delta_e$, and thrust $T$ that satisfy steady-level flight:

$$\dot{w} = \dot{q} = \dot{h} = 0 \quad \Rightarrow \quad L = W, \quad T = D$$

### Algorithm:

1. **Initial guess** for elevator deflection, pitch angle, and thrust
2. **Propagate dynamics** → compute residuals (w_dot, q_dot, h_dot)
3. **Newton iteration** with finite-difference Jacobian
4. **Return** trimmed state and control inputs

---

## UDP Data Interface

FlightGear reads real-time data using the **Generic protocol**. The Python simulation sends comma-separated values matching `myproto.xml`.

### Data Format:

| Field | FlightGear Property              | Units   |
| ----- | --------------------------------- | ------- |
| 1     | `/position/latitude-deg`          | degrees |
| 2     | `/position/longitude-deg`        | degrees |
| 3     | `/position/altitude-ft`           | feet    |
| 4     | `/orientation/roll-deg`           | degrees |
| 5     | `/orientation/pitch-deg`          | degrees |
| 6     | `/orientation/heading-deg`        | degrees |

### Data Flow:

```
Python → UDP → FlightGear → Visualization
 |         |        |
 |         |        └──► 3D model updates
 |         └────────────► UDP socket (127.0.0.1:5500)
 └──────────────────────► Generates comma-separated lines
```

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

3. **Place `myproto.xml` in FlightGear protocols directory**:
   ```
   C:\Users\<YourUser>\FlightGear\fgdata\Protocols\myproto.xml
   ```

4. **Update paths in batch files**:
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

## Future Improvements

* **Wind-axis equations** and advanced atmosphere models (winds, turbulence)
* **Sensor models** (IMU, GPS, air data) with EKF fusion
* **Bidirectional UDP link** (Python ←→ FlightGear for controls)
* **Autopilot implementation** (altitude hold, heading hold, etc.)
* **Reinforcement Learning** integration for flight control
* **Data logging** (CSV/JSON export for post-flight analysis)
* **Real-time plotting** (live graphs of state variables)
* **Multi-aircraft support** for formation flight simulations

---

## Technical References

* **Aircraft Dynamics**: Stevens, B. L., & Lewis, F. L. (2003). *Aircraft Control and Simulation*
* **FlightGear Documentation**: [https://wiki.flightgear.org/](https://wiki.flightgear.org/)
* **WGS-84**: NIMA (2000). *Department of Defense World Geodetic System 1984*
* **ISA Atmosphere**: ISO 2533:1975
