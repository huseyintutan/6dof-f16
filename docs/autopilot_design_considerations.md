# Autopilot Design: Python vs MATLAB Simulink

## Short Answer: **NO, MATLAB Simulink is NOT required**

You can build a complete autopilot system using Python with the existing simulation framework.

---

## Architecture Comparison

### Option 1: Pure Python (Current Framework)
✅ **Advantages:**
- **Already have the simulation** - 6-DoF dynamics, sensors, real-time loop
- **Free and open-source** - No license costs
- **Flexible** - Easy to integrate with web dashboards, data logging, AI/ML
- **Real-time capable** - Your current loop runs at 100 Hz internally
- **Full control** - Can implement any control algorithm
- **Easy debugging** - Standard Python tools, print statements, profilers
- **Integration** - Works with FlightGear, web dashboards, sensor fusion

⚠️ **Considerations:**
- Need to implement control algorithms manually (but libraries exist)
- No visual block diagrams (but code is more flexible)
- Need to handle real-time scheduling yourself (but you already do this)

### Option 2: MATLAB Simulink
✅ **Advantages:**
- **Visual block diagrams** - Easy to see control structure
- **Pre-built blocks** - PID controllers, filters, state machines
- **Code generation** - Can generate C/C++ code for real-time systems
- **Industry standard** - Used in aerospace industry
- **Extensive toolboxes** - Control System Toolbox, Aerospace Blockset

❌ **Disadvantages:**
- **Expensive** - MATLAB + Simulink + toolboxes = $$$
- **Integration complexity** - Need to interface with Python simulation
- **Less flexible** - Block-based approach can be limiting
- **Learning curve** - Different paradigm than code
- **Deployment** - Need MATLAB Runtime for deployment

---

## Autopilot Implementation Options

### 1. Pure Python Implementation (Recommended)

You already have everything needed:

```python
# Example structure (pseudocode)
class Autopilot:
    def __init__(self):
        self.altitude_hold = AltitudeHoldController()
        self.heading_hold = HeadingHoldController()
        self.speed_hold = SpeedHoldController()
        self.attitude_hold = AttitudeHoldController()
    
    def update(self, t, state, sensors):
        # Get desired values from pilot/waypoints
        h_desired = 3048.0  # m
        psi_desired = 45.0  # deg
        V_desired = 150.0   # m/s
        
        # Compute control outputs
        controls = self.compute_controls(
            state, sensors,
            h_desired, psi_desired, V_desired
        )
        return controls
```

**Control Algorithms Available in Python:**
- **PID controllers**: `scipy.signal`, `control` library
- **State space**: `scipy.linalg`, `control`
- **LQR/LQG**: `control` library
- **Extended Kalman Filter**: `filterpy`, `scipy`
- **Path planning**: `scipy`, `numpy`

### 2. Hybrid: Python + Control Libraries

Use Python for simulation, leverage control libraries:

```python
# Example using python-control library
from control import tf, pid, feedback
import numpy as np

# PID controller
Kp, Ki, Kd = 1.0, 0.1, 0.01
pid_controller = pid(Kp, Ki, Kd)

# Or state-space controller
from control import ss, lqr
A = np.array([[...]])  # State matrix
B = np.array([[...]])   # Input matrix
Q = np.eye(4)           # State weight
R = np.eye(2)           # Input weight
K, S, E = lqr(A, B, Q, R)
```

### 3. MATLAB Simulink Integration (If Needed)

If you want to use Simulink for design:

**Option A: Design in Simulink, Export to Python**
1. Design autopilot in Simulink
2. Generate C code from Simulink
3. Wrap C code with Python (using `ctypes` or `Cython`)
4. Integrate with your simulation

**Option B: Co-simulation**
1. Python runs aircraft dynamics
2. Simulink runs autopilot
3. Communicate via UDP/TCP (slower, more complex)

---

## What You Need for Autopilot

### 1. Control Algorithms (Available in Python)

#### PID Controllers
```python
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.last_error = 0.0
    
    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output
```

#### State Space Controllers
- Use `python-control` library
- LQR (Linear Quadratic Regulator)
- LQG (Linear Quadratic Gaussian)
- State feedback controllers

#### Filtering
- Kalman filters: `filterpy` library
- Low-pass filters: `scipy.signal`
- Sensor fusion: Custom implementation

### 2. Autopilot Modes

You can implement:

1. **Altitude Hold** (ALT HOLD)
   - Input: Desired altitude
   - Output: Elevator command
   - Uses: Barometric altimeter or GPS

2. **Heading Hold** (HDG HOLD)
   - Input: Desired heading
   - Output: Aileron + Rudder commands
   - Uses: Magnetometer or GPS heading

3. **Speed Hold** (SPD HOLD)
   - Input: Desired airspeed
   - Output: Throttle command
   - Uses: Pitot-static airspeed

4. **Attitude Hold** (ATT HOLD)
   - Input: Desired roll/pitch
   - Output: Aileron/Elevator commands
   - Uses: IMU (gyroscope, accelerometer)

5. **Autopilot Modes**
   - **AP1**: Altitude + Heading hold
   - **AP2**: Altitude + Speed hold
   - **AP3**: Full autopilot (altitude + heading + speed)
   - **Waypoint Navigation**: Follow GPS waypoints

### 3. Sensor Fusion (You Already Have!)

Your `SensorSuite` provides:
- IMU (accelerometer, gyroscope)
- GPS (position, velocity, heading)
- Air Data (airspeed, altitude, α, β)
- Magnetometer (heading)

You can fuse these with:
- **Complementary Filter**: Simple, fast
- **Extended Kalman Filter (EKF)**: More accurate, handles sensor noise
- **Unscented Kalman Filter (UKF)**: Better for non-linear systems

---

## Recommended Approach

### Phase 1: Basic Autopilot (Python)
1. **Speed Hold Controller** (PID)
   - Input: Current airspeed, desired airspeed
   - Output: Throttle adjustment
   - Simple to implement, useful for testing

2. **Altitude Hold Controller** (PID)
   - Input: Current altitude, desired altitude
   - Output: Elevator command
   - Uses barometric altimeter or GPS

3. **Heading Hold Controller** (PID)
   - Input: Current heading, desired heading
   - Output: Aileron + Rudder commands
   - Uses magnetometer or GPS heading

### Phase 2: Advanced Features
1. **State Estimation** (EKF)
   - Fuse IMU, GPS, Air Data
   - Better state estimates than individual sensors

2. **Mode Switching**
   - AP1, AP2, AP3 modes
   - Smooth transitions between modes

3. **Waypoint Navigation**
   - Follow GPS waypoints
   - Turn coordination

### Phase 3: Advanced Control
1. **LQR Controller** (full state feedback)
2. **Model Predictive Control (MPC)**
3. **Adaptive Control** (handles parameter uncertainty)

---

## Python Libraries for Control Systems

### Recommended Libraries:
1. **`python-control`** (Control System Toolbox)
   - PID controllers
   - State space systems
   - LQR, LQG controllers
   - Frequency response analysis

2. **`filterpy`** (Kalman Filtering)
   - Kalman filters
   - Extended Kalman Filter
   - Unscented Kalman Filter
   - Sensor fusion

3. **`scipy`** (Scientific Computing)
   - Signal processing
   - Optimization
   - Linear algebra

4. **`numpy`** (Already using)
   - Matrix operations
   - Linear algebra

---

## Example: Simple Autopilot Structure

```python
class Autopilot:
    def __init__(self):
        # Controllers
        self.altitude_controller = PIDController(Kp=0.5, Ki=0.01, Kd=0.1)
        self.heading_controller = PIDController(Kp=0.3, Ki=0.005, Kd=0.05)
        self.speed_controller = PIDController(Kp=100.0, Ki=10.0, Kd=50.0)
        
        # Desired values
        self.h_desired = 3048.0  # m
        self.psi_desired = 0.0   # deg
        self.V_desired = 150.0   # m/s
        
        # Base trim controls
        self.controls_base = None
        
    def update(self, t, state, sensors, controls_base):
        self.controls_base = controls_base.copy()
        controls = controls_base.copy()
        
        # Altitude hold
        h_error = self.h_desired - state["h"]
        de_alt = self.altitude_controller.update(h_error, DT)
        controls["de"] += de_alt
        
        # Heading hold
        psi_error = self.psi_desired - math.degrees(state["psi"])
        # Normalize to [-180, 180]
        while psi_error > 180: psi_error -= 360
        while psi_error < -180: psi_error += 360
        
        da_hdg = self.heading_controller.update(psi_error, DT)
        controls["da"] += da_hdg
        
        # Speed hold
        V_error = self.V_desired - state["V"]
        thrust_adjustment = self.speed_controller.update(V_error, DT)
        # Would adjust throttle here (if implemented)
        
        return controls
```

---

## Conclusion

**You DON'T need MATLAB Simulink** to build an autopilot. Your Python simulation framework is sufficient and actually more flexible.

### When to Consider Simulink:
- **Design phase**: If you want visual block diagrams
- **Industry requirement**: If client/employer requires Simulink
- **Code generation**: If you need to generate C/C++ for real-time systems
- **Team familiarity**: If your team only knows Simulink

### When Python is Better:
- **Development**: Faster iteration, easier debugging
- **Integration**: Web dashboards, data logging, AI/ML
- **Cost**: Free and open-source
- **Flexibility**: Any algorithm, any library
- **Deployment**: Easy to deploy on any system

---

## Recommendation

**Start with Python** - you can always export to Simulink later if needed. The control algorithms are the same regardless of implementation language.

**Next Steps:**
1. Implement basic PID controllers (1-2 hours)
2. Add speed hold autopilot (1 hour)
3. Add altitude hold (1 hour)
4. Add heading hold (1 hour)
5. Combine into full autopilot (2-3 hours)

**Total time: ~6-8 hours for a basic autopilot** (vs. days of learning Simulink)

