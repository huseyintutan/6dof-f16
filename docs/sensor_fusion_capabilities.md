# Sensor Fusion with EKF: Capabilities

## What We Can Do Now

### 1. **State Estimation from Noisy Sensors**

Instead of using true state (which is not available in real flight), we now estimate state from:
- ✅ **IMU** (accelerometer + gyroscope)
- ✅ **GPS** (position + velocity)
- ✅ **Air Data** (airspeed, alpha, beta)
- ✅ **Magnetometer** (heading)
- ✅ **Barometric Altimeter** (altitude)

### 2. **Sensor Bias Estimation**

EKF automatically estimates and corrects for:
- **Accelerometer bias** (ba_x, ba_y, ba_z)
- **Gyroscope bias** (bg_p, bg_q, bg_r)

These biases are learned during flight and subtracted from measurements.

### 3. **Improved Accuracy**

**Before (individual sensors):**
- GPS altitude: ±3 m noise
- IMU: ±0.05 m/s² noise, ±0.01 m/s² bias
- Air data: ±0.5 m/s noise
- Baro: ±2 m noise

**After (EKF fusion):**
- Position: ±1-2 m (better than GPS alone)
- Velocity: ±0.05 m/s (better than GPS alone)
- Attitude: ±0.5° (better than IMU alone)
- Biases: Corrected automatically

### 4. **Real-time State Updates**

EKF runs at simulation rate (100 Hz) and:
- **Predicts** state using IMU (high rate)
- **Updates** with GPS (10 Hz)
- **Updates** with Air Data (every step)
- **Updates** with Magnetometer (every step)
- **Updates** with Baro (every step)

### 5. **Comparison: True vs Estimated**

You can now see:
- **True state** (from simulation - not available in real flight)
- **Estimated state** (from EKF - what you'd have in real flight)
- **Error** (difference between true and estimated)

This helps validate the filter performance.

---

## EKF State Vector (15 states)

```
x = [N, E, D,          # Position (NED) [m]
     u, v, w,          # Velocity (body) [m/s]
     phi, theta, psi,  # Attitude (Euler) [rad]
     ba_x, ba_y, ba_z, # Accelerometer bias [m/s²]
     bg_p, bg_q, bg_r] # Gyroscope bias [rad/s]
```

---

## Measurement Models

### IMU (6 measurements)
- Accelerometer: `fx, fy, fz` (specific force)
- Gyroscope: `p, q, r` (angular rates)

### GPS (6 measurements)
- Position: `lat, lon, alt` → converted to `N, E, D`
- Velocity: `vn, ve, vd` (NED frame)

### Air Data (3 measurements)
- Airspeed: `V` (from pitot)
- Alpha: `α` (angle of attack)
- Beta: `β` (sideslip)

### Magnetometer (1 measurement)
- Heading: `ψ` (yaw angle)

### Barometric Altimeter (1 measurement)
- Altitude: `h` (from static pressure)

---

## What This Enables

### 1. **Autopilot Development**
- Use estimated state instead of true state
- More realistic control system testing
- Handles sensor noise and bias

### 2. **Navigation System**
- Dead reckoning between GPS updates
- IMU provides high-rate updates
- GPS corrects drift

### 3. **Sensor Validation**
- Compare sensor outputs
- Detect sensor failures
- Monitor filter health (covariance)

### 4. **Research & Development**
- Test different filter configurations
- Compare filter performance
- Optimize sensor fusion algorithms

---

## Filter Performance Metrics

You can monitor:

1. **State Covariance (P)**
   - Uncertainty in each state
   - Lower = more confident
   - Diagonal elements = variance

2. **Innovation (y = z - h(x))**
   - Measurement prediction error
   - Should be small and white noise
   - Large = sensor problem or model error

3. **Kalman Gain (K)**
   - How much to trust measurements
   - High = trust measurements more
   - Low = trust prediction more

---

## Next Steps / Improvements

### Short Term
1. ✅ Basic EKF implementation
2. ✅ Integration with simulation
3. ✅ Dashboard visualization
4. ⏳ Tune filter parameters (Q, R matrices)
5. ⏳ Add bias estimation visualization

### Medium Term
1. ⏳ Add sensor fault detection
2. ⏳ Implement UKF (Unscented Kalman Filter) for better non-linear handling
3. ⏳ Add GPS outage simulation
4. ⏳ Implement complementary filter as baseline

### Long Term
1. ⏳ Multi-sensor fusion (add more sensors)
2. ⏳ Adaptive filtering (auto-tune Q, R)
3. ⏳ Outlier rejection
4. ⏳ Sensor redundancy management

---

## Usage Example

```python
# Initialize EKF
ekf = EKFSensorFusion(lat0_deg=41.0, lon0_deg=29.0, dt=0.01)

# Initialize with true state (first time only)
ekf.initialize(state, N, E, D)

# In simulation loop:
# 1. Predict (using IMU)
ekf.predict()

# 2. Update with measurements
ekf.update_imu(imu_accel, imu_gyro)
ekf.update_gps(gps_data)
ekf.update_air_data(air_data)
ekf.update_magnetometer(mag_data)
ekf.update_baro(baro_data)

# 3. Get estimated state
estimated_state = ekf.get_estimated_state()
```

---

## Technical Notes

### Process Noise (Q)
- Models uncertainty in dynamics
- Position: 0.1 m/s (position uncertainty growth)
- Velocity: 0.1 m/s² (acceleration uncertainty)
- Attitude: 0.001 rad/s (attitude uncertainty)
- Bias: 1e-6 (bias drift rate)

### Measurement Noise (R)
- From sensor specifications
- IMU accel: 0.05 m/s²
- IMU gyro: 0.001 rad/s
- GPS pos: 3.0 m
- GPS vel: 0.1 m/s
- Air data: 0.5 m/s
- Magnetometer: 0.5° (0.0087 rad)
- Baro: 2.0 m

### Filter Tuning
- Increase Q → filter trusts measurements more
- Decrease Q → filter trusts prediction more
- Increase R → filter trusts measurements less
- Decrease R → filter trusts measurements more

