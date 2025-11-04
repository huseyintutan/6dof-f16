# f16_sensors.py
# -------------------------------------------------------------
# Sensor models for F-16 flight simulation
# 
# Provides realistic sensor outputs with:
#   - Noise (Gaussian white noise)
#   - Bias (constant offsets)
#   - Scale factors (non-linear errors)
#   - Sampling delays (optional)
#
# Sensors included:
#   1. IMU (Inertial Measurement Unit)
#      - Accelerometer (3-axis, body frame)
#      - Gyroscope (3-axis, body frame)
#   2. GPS (Global Positioning System)
#      - Position (lat, lon, alt)
#      - Velocity (NED frame)
#   3. Air Data Sensors
#      - Pitot-static (airspeed, altitude)
#      - Angle of Attack (alpha)
#      - Sideslip (beta)
#   4. Magnetometer (heading)
#   5. Barometric Altimeter
#
# Usage:
#   from src.f16_sensors import SensorSuite
#   sensors = SensorSuite(enable_noise=True, enable_bias=True)
#   measurements = sensors.update(state, N, E, D, lat0, lon0, dt)
# -------------------------------------------------------------

import math
import numpy as np
from typing import Dict, Optional
from .f16_atmosphere import get_atmosphere
from .f16_forces import uvw_to_alphabeta
from .f16_kinematics import dcm_body_to_ned


class WhiteNoiseGenerator:
    """Simple white noise generator with optional seed for reproducibility."""
    def __init__(self, seed: Optional[int] = None):
        self.rng = np.random.RandomState(seed) if seed is not None else np.random
    
    def generate(self, std: float, shape: tuple = ()) -> np.ndarray:
        """Generate white noise with standard deviation std."""
        return self.rng.normal(0.0, std, shape)


class SensorSuite:
    """
    Comprehensive sensor suite for F-16 flight simulation.
    
    All sensors can be individually enabled/disabled and configured.
    """
    
    def __init__(self,
                 enable_noise: bool = True,
                 enable_bias: bool = True,
                 seed: Optional[int] = None,
                 # IMU parameters
                 accel_noise_std: float = 0.05,      # m/s²
                 accel_bias: tuple = (0.01, 0.01, 0.01),  # m/s²
                 gyro_noise_std: float = 0.001,      # rad/s
                 gyro_bias: tuple = (0.0001, 0.0001, 0.0001),  # rad/s
                 # GPS parameters
                 gps_pos_noise_std: float = 3.0,     # meters
                 gps_vel_noise_std: float = 0.1,     # m/s
                 gps_update_rate: float = 10.0,      # Hz
                 gps_delay: float = 0.1,             # seconds
                 # Air data parameters
                 pitot_noise_std: float = 0.5,       # m/s
                 static_noise_std: float = 1.0,      # meters
                 alpha_noise_std: float = 0.1,       # degrees
                 beta_noise_std: float = 0.1,        # degrees
                 # Magnetometer parameters
                 mag_noise_std: float = 0.5,         # degrees
                 mag_bias: float = 2.0,             # degrees
                 # Barometric altimeter parameters
                 baro_noise_std: float = 2.0,        # meters
                 baro_bias: float = 0.0,            # meters
                 ):
        """
        Initialize sensor suite.
        
        Args:
            enable_noise: Enable noise on all sensors
            enable_bias: Enable bias on sensors
            seed: Random seed for reproducibility
            accel_noise_std: Accelerometer noise standard deviation [m/s²]
            accel_bias: Accelerometer bias (x, y, z) [m/s²]
            gyro_noise_std: Gyroscope noise standard deviation [rad/s]
            gyro_bias: Gyroscope bias (x, y, z) [rad/s]
            gps_pos_noise_std: GPS position noise [m]
            gps_vel_noise_std: GPS velocity noise [m/s]
            gps_update_rate: GPS update rate [Hz]
            gps_delay: GPS delay [s]
            pitot_noise_std: Pitot airspeed noise [m/s]
            static_noise_std: Static pressure altitude noise [m]
            alpha_noise_std: Alpha sensor noise [deg]
            beta_noise_std: Beta sensor noise [deg]
            mag_noise_std: Magnetometer noise [deg]
            mag_bias: Magnetometer bias [deg]
            baro_noise_std: Barometric altimeter noise [m]
            baro_bias: Barometric altimeter bias [m]
        """
        self.enable_noise = enable_noise
        self.enable_bias = enable_bias
        self.noise_gen = WhiteNoiseGenerator(seed)
        
        # IMU parameters
        self.accel_noise_std = accel_noise_std
        self.accel_bias = np.array(accel_bias, dtype=float)
        self.gyro_noise_std = gyro_noise_std
        self.gyro_bias = np.array(gyro_bias, dtype=float)
        
        # GPS parameters
        self.gps_pos_noise_std = gps_pos_noise_std
        self.gps_vel_noise_std = gps_vel_noise_std
        self.gps_update_rate = gps_update_rate
        self.gps_dt = 1.0 / gps_update_rate if gps_update_rate > 0 else 1.0
        self.gps_delay = gps_delay
        self.gps_last_update = -self.gps_dt
        self.gps_delayed_data = None
        
        # Air data parameters
        self.pitot_noise_std = pitot_noise_std
        self.static_noise_std = static_noise_std
        self.alpha_noise_std = alpha_noise_std
        self.beta_noise_std = beta_noise_std
        
        # Magnetometer parameters
        self.mag_noise_std = mag_noise_std
        self.mag_bias = mag_bias
        
        # Barometric altimeter parameters
        self.baro_noise_std = baro_noise_std
        self.baro_bias = baro_bias
        
        # Time tracking
        self.t = 0.0
        
        # GPS delay buffer
        self.gps_buffer = []
    
    def update(self,
               t: float,
               state: Dict,
               N: float,
               E: float,
               D: float,
               lat0_deg: float,
               lon0_deg: float,
               dt: float,
               forces: Optional[Dict] = None) -> Dict:
        """
        Update all sensors and return measurements.
        
        Args:
            t: Current simulation time [s]
            state: State dictionary with keys: u, v, w, p, q, r, phi, theta, psi, h
            N, E, D: NED position [m]
            lat0_deg, lon0_deg: Reference geodetic coordinates [deg]
            dt: Time step [s]
            forces: Optional forces dict (Fx, Fy, Fz) for accelerometer
        
        Returns:
            Dictionary with all sensor measurements
        """
        self.t = t
        
        # Extract state variables
        u = float(state["u"])
        v = float(state["v"])
        w = float(state["w"])
        p = float(state["p"])
        q = float(state["q"])
        r = float(state["r"])
        phi = float(state["phi"])
        theta = float(state["theta"])
        psi = float(state["psi"])
        h = float(state["h"])
        
        # Initialize measurements dictionary
        measurements = {}
        
        # 1. IMU - Accelerometer
        measurements["imu_accel"] = self._imu_accelerometer(
            u, v, w, p, q, r, phi, theta, psi, h, forces
        )
        
        # 2. IMU - Gyroscope
        measurements["imu_gyro"] = self._imu_gyroscope(p, q, r)
        
        # 3. GPS
        measurements["gps"] = self._gps(
            t, N, E, D, lat0_deg, lon0_deg, u, v, w, phi, theta, psi, dt
        )
        
        # 4. Air Data Sensors
        measurements["air_data"] = self._air_data(u, v, w, h)
        
        # 5. Magnetometer
        measurements["magnetometer"] = self._magnetometer(psi)
        
        # 6. Barometric Altimeter
        measurements["baro_alt"] = self._barometric_altimeter(h)
        
        return measurements
    
    def _imu_accelerometer(self, u, v, w, p, q, r, phi, theta, psi, h, forces=None):
        """
        Compute accelerometer measurements (body-axis specific force).
        
        True specific force: f = a_body - g_body
        where a_body is acceleration in body frame and g_body is gravity in body frame.
        """
        # Get gravity in body frame
        R_bn = dcm_body_to_ned(phi, theta, psi)
        g_ned = np.array([0.0, 0.0, 9.80665])  # Standard gravity, down is positive
        g_body = R_bn.T @ g_ned
        
        # If forces provided, compute acceleration from F = m*a
        # Otherwise, use approximate acceleration from kinematics
        if forces is not None:
            m = 9300.0  # F-16 mass [kg]
            Fx = float(forces.get("Fx", 0.0))
            Fy = float(forces.get("Fy", 0.0))
            Fz = float(forces.get("Fz", 0.0))
            # Remove gravity component (already included in forces)
            a_body = np.array([Fx, Fy, Fz], dtype=float) / m
            # Specific force = acceleration - gravity (in body frame)
            f_body = a_body - g_body
        else:
            # Approximate: use kinematic acceleration (centrifugal terms)
            # f ≈ -omega x V - g_body
            omega = np.array([p, q, r], dtype=float)
            V_body = np.array([u, v, w], dtype=float)
            f_body = -np.cross(omega, V_body) - g_body
        
        # Add noise and bias
        if self.enable_noise:
            noise = self.noise_gen.generate(self.accel_noise_std, (3,))
            f_body += noise
        
        if self.enable_bias:
            f_body += self.accel_bias
        
        return {
            "fx": float(f_body[0]),
            "fy": float(f_body[1]),
            "fz": float(f_body[2]),
            "units": "m/s²"
        }
    
    def _imu_gyroscope(self, p, q, r):
        """Compute gyroscope measurements (body-axis angular rates)."""
        omega = np.array([p, q, r], dtype=float)
        
        # Add noise and bias
        if self.enable_noise:
            noise = self.noise_gen.generate(self.gyro_noise_std, (3,))
            omega += noise
        
        if self.enable_bias:
            omega += self.gyro_bias
        
        return {
            "p": float(omega[0]),
            "q": float(omega[1]),
            "r": float(omega[2]),
            "units": "rad/s"
        }
    
    def _gps(self, t, N, E, D, lat0, lon0, u, v, w, phi, theta, psi, dt):
        """
        Compute GPS measurements (position and velocity in geodetic and NED).
        
        GPS has update rate and delay modeling.
        """
        # Convert NED to geodetic (simplified, assumes small area)
        # Better accuracy would use proper WGS84 ellipsoid
        lat_deg = lat0 + (N / 111320.0)
        lon_deg = lon0 + (E / (111320.0 * math.cos(math.radians(lat0))))
        alt_m = -D
        
        # Convert body velocity to NED velocity
        R_bn = dcm_body_to_ned(phi, theta, psi)
        V_body = np.array([u, v, w], dtype=float)
        V_ned = R_bn @ V_body
        
        # Check if GPS should update (based on update rate)
        should_update = (t - self.gps_last_update) >= self.gps_dt
        
        if should_update:
            # Add noise to true values
            lat_noisy = lat_deg
            lon_noisy = lon_deg
            alt_noisy = alt_m
            vn_noisy = V_ned[0]
            ve_noisy = V_ned[1]
            vd_noisy = V_ned[2]
            
            if self.enable_noise:
                # Position noise (convert meters to degrees)
                lat_noise_m = self.noise_gen.generate(self.gps_pos_noise_std)
                lon_noise_m = self.noise_gen.generate(self.gps_pos_noise_std)
                alt_noise = self.noise_gen.generate(self.gps_pos_noise_std)
                
                lat_noisy += lat_noise_m / 111320.0
                lon_noisy += lon_noise_m / (111320.0 * math.cos(math.radians(lat0)))
                alt_noisy += alt_noise
                
                # Velocity noise
                vn_noisy += self.noise_gen.generate(self.gps_vel_noise_std)
                ve_noisy += self.noise_gen.generate(self.gps_vel_noise_std)
                vd_noisy += self.noise_gen.generate(self.gps_vel_noise_std)
            
            # Store with timestamp for delay modeling
            gps_data = {
                "timestamp": t,
                "lat_deg": lat_noisy,
                "lon_deg": lon_noisy,
                "alt_m": alt_noisy,
                "vn_mps": vn_noisy,
                "ve_mps": ve_noisy,
                "vd_mps": vd_noisy,
            }
            
            # Add to delay buffer
            self.gps_buffer.append(gps_data)
            self.gps_last_update = t
        
        # Return delayed GPS data (if available)
        if self.gps_buffer:
            # Find data from approximately (t - delay) ago
            target_time = t - self.gps_delay
            best_data = None
            best_diff = float('inf')
            
            for data in self.gps_buffer:
                diff = abs(data["timestamp"] - target_time)
                if diff < best_diff:
                    best_diff = diff
                    best_data = data
            
            # Clean old data from buffer
            self.gps_buffer = [d for d in self.gps_buffer if d["timestamp"] > t - self.gps_delay - 1.0]
            
            if best_data:
                return {
                    "lat_deg": best_data["lat_deg"],
                    "lon_deg": best_data["lon_deg"],
                    "alt_m": best_data["alt_m"],
                    "vn_mps": best_data["vn_mps"],
                    "ve_mps": best_data["ve_mps"],
                    "vd_mps": best_data["vd_mps"],
                    "units": {"pos": "deg/m", "vel": "m/s"}
                }
        
        # No GPS data available yet (return current noisy if needed)
        return {
            "lat_deg": lat_deg,
            "lon_deg": lon_deg,
            "alt_m": alt_m,
            "vn_mps": float(V_ned[0]),
            "ve_mps": float(V_ned[1]),
            "vd_mps": float(V_ned[2]),
            "units": {"pos": "deg/m", "vel": "m/s"}
        }
    
    def _air_data(self, u, v, w, h):
        """Compute air data sensor measurements (pitot, static, alpha, beta)."""
        # True airspeed, alpha, beta
        alpha_deg_true, beta_deg_true, V_true = uvw_to_alphabeta(u, v, w)
        
        # Pitot-static system
        # True airspeed from pitot
        V_pitot = V_true
        if self.enable_noise:
            V_pitot += self.noise_gen.generate(self.pitot_noise_std)
        V_pitot = max(0.0, V_pitot)  # Airspeed can't be negative
        
        # Static pressure altitude
        # Use ISA model to get static pressure
        atm = get_atmosphere(h)
        p_static = atm["p"]
        
        # Invert ISA to get altitude from pressure (simplified)
        # ISA: p = p0 * (T/T0)^(-g/(aL*R)), T = T0 + aL*h
        # For small h, approximate: h ≈ (p0 - p) / (rho0 * g)
        # More accurate: use inverse of ISA model
        h_static = h
        if self.enable_noise:
            h_static += self.noise_gen.generate(self.static_noise_std)
        
        # Alpha and beta sensors
        alpha_meas = alpha_deg_true
        beta_meas = beta_deg_true
        
        if self.enable_noise:
            alpha_meas += self.noise_gen.generate(self.alpha_noise_std)
            beta_meas += self.noise_gen.generate(self.beta_noise_std)
        
        return {
            "airspeed_mps": V_pitot,
            "static_alt_m": h_static,
            "alpha_deg": alpha_meas,
            "beta_deg": beta_meas,
            "units": {"airspeed": "m/s", "alt": "m", "angles": "deg"}
        }
    
    def _magnetometer(self, psi):
        """Compute magnetometer heading measurement."""
        heading_deg = math.degrees(psi) % 360.0
        
        if self.enable_noise:
            heading_deg += self.noise_gen.generate(self.mag_noise_std)
        
        if self.enable_bias:
            heading_deg += self.mag_bias
        
        heading_deg = heading_deg % 360.0
        
        return {
            "heading_deg": heading_deg,
            "units": "deg"
        }
    
    def _barometric_altimeter(self, h):
        """Compute barometric altimeter measurement."""
        h_baro = h
        
        if self.enable_noise:
            h_baro += self.noise_gen.generate(self.baro_noise_std)
        
        if self.enable_bias:
            h_baro += self.baro_bias
        
        return {
            "alt_m": h_baro,
            "units": "m"
        }
