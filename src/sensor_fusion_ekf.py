# sensor_fusion_ekf.py
# Extended Kalman Filter (EKF) for sensor fusion
# Fuses IMU, GPS, Air Data, Magnetometer, and Barometric Altimeter

import numpy as np
import math
from typing import Dict, Optional, Tuple
from src.f16_kinematics import dcm_body_to_ned, quat_to_euler, euler_to_quat
from src.f16_atmosphere import get_atmosphere

# Helper function for density (compatible with f16_forces)
def rho_at(h_m: float) -> float:
    """Get air density at altitude h_m [m]."""
    return get_atmosphere(h_m)["rho"]


class EKFSensorFusion:
    """
    Extended Kalman Filter for aircraft state estimation.
    
    State vector (15 states):
        x = [N, E, D,          # Position (NED) [m]
             u, v, w,          # Velocity (body) [m/s]
             phi, theta, psi,  # Attitude (Euler) [rad]
             ba_x, ba_y, ba_z, # Accelerometer bias [m/s²]
             bg_p, bg_q, bg_r] # Gyroscope bias [rad/s]
    
    Measurements:
        - IMU: Accelerometer (fx, fy, fz), Gyroscope (p, q, r)
        - GPS: Position (lat, lon, alt), Velocity (NED)
        - Air Data: Airspeed, Alpha, Beta
        - Magnetometer: Heading
        - Barometric Altimeter: Altitude
    """
    
    def __init__(self,
                 lat0_deg: float = 41.0,
                 lon0_deg: float = 29.0,
                 dt: float = 0.01,
                 # Process noise (process uncertainty)
                 pos_noise: float = 0.1,      # m/s
                 vel_noise: float = 0.1,      # m/s²
                 att_noise: float = 0.001,    # rad/s
                 bias_noise: float = 1e-6,     # bias drift
                 # Measurement noise (from sensor specs)
                 imu_accel_noise: float = 0.05,   # m/s²
                 imu_gyro_noise: float = 0.001,   # rad/s
                 gps_pos_noise: float = 3.0,      # m
                 gps_vel_noise: float = 0.1,      # m/s
                 air_data_noise: float = 0.5,     # m/s
                 mag_noise: float = 0.5,          # deg
                 baro_noise: float = 2.0):         # m
        """
        Initialize EKF for sensor fusion.
        
        Args:
            lat0_deg, lon0_deg: Reference geodetic coordinates
            dt: Time step [s]
            *_noise: Process and measurement noise standard deviations
        """
        self.lat0_deg = lat0_deg
        self.lon0_deg = lon0_deg
        self.dt = dt
        
        # State dimension
        self.nx = 15
        
        # Process noise covariance
        self.Q = np.diag([
            pos_noise**2, pos_noise**2, pos_noise**2,  # Position
            vel_noise**2, vel_noise**2, vel_noise**2,  # Velocity
            att_noise**2, att_noise**2, att_noise**2,  # Attitude
            bias_noise**2, bias_noise**2, bias_noise**2,  # Accel bias
            bias_noise**2, bias_noise**2, bias_noise**2   # Gyro bias
        ])
        
        # Measurement noise covariance (will be updated per measurement)
        self.R_imu = np.diag([imu_accel_noise**2] * 3 + [imu_gyro_noise**2] * 3)
        self.R_gps = np.diag([gps_pos_noise**2] * 3 + [gps_vel_noise**2] * 3)
        self.R_air = np.diag([air_data_noise**2] * 3)  # V, alpha, beta
        self.R_mag = np.array([[mag_noise**2]])  # heading
        self.R_baro = np.array([[baro_noise**2]])  # altitude
        
        # State covariance
        self.P = np.eye(self.nx) * 100.0  # Initial uncertainty (large)
        
        # State (initialized later) - ensure it's a 1D array
        self.x = np.zeros(self.nx, dtype=np.float64)
        self.initialized = False
        
        # Gravity
        self.g = 9.80665  # m/s²
        
        # Mass (for accelerometer)
        self.m = 9300.0  # kg (F-16)
        
        # Earth radius (approximate, for NED conversion)
        self.R_earth = 6378137.0  # m (WGS84)
        self.deg_to_rad = math.pi / 180.0
        self.rad_to_deg = 180.0 / math.pi
    
    def initialize(self, state: Dict, N: float, E: float, D: float):
        """
        Initialize EKF state from true state.
        
        Args:
            state: True state dict
            N, E, D: NED position [m]
        """
        self.x[0] = N  # N
        self.x[1] = E  # E
        self.x[2] = D  # D
        self.x[3] = state["u"]  # u
        self.x[4] = state["v"]  # v
        self.x[5] = state["w"]  # w
        self.x[6] = state["phi"]  # phi
        self.x[7] = state["theta"]  # theta
        self.x[8] = state["psi"]  # psi
        # Biases initialized to zero
        self.x[9:12] = 0.0   # accel bias
        self.x[12:15] = 0.0  # gyro bias
        
        # Initialize covariance (smaller for known initial state)
        self.P = np.eye(self.nx) * 1.0
        self.P[0:3, 0:3] *= 10.0  # Position uncertainty
        self.P[3:6, 3:6] *= 0.1   # Velocity uncertainty
        self.P[6:9, 6:9] *= 0.01  # Attitude uncertainty
        self.P[9:15, 9:15] *= 0.01  # Bias uncertainty
        
        self.initialized = True
    
    def predict(self, controls: Optional[Dict] = None):
        """
        Predict step: Propagate state using IMU measurements.
        
        Args:
            controls: Control inputs (optional, for future use)
        """
        if not self.initialized:
            return
        
        # Extract state
        N, E, D = self.x[0], self.x[1], self.x[2]
        u, v, w = self.x[3], self.x[4], self.x[5]
        phi, theta, psi = self.x[6], self.x[7], self.x[8]
        ba = self.x[9:12]  # accel bias
        bg = self.x[12:15]  # gyro bias
        
        # Get IMU measurements (should be passed in update, but for now assume stored)
        # For prediction, we use the last IMU measurements
        if not hasattr(self, 'last_imu_accel'):
            return  # Can't predict without IMU
        
        # Use last IMU measurements (corrected for bias)
        f_measured = self.last_imu_accel - ba
        omega_measured = self.last_imu_gyro - bg
        
        # State derivative
        x_dot = self._state_derivative(
            N, E, D, u, v, w, phi, theta, psi,
            f_measured, omega_measured
        )
        
        # Predict state
        self.x += x_dot * self.dt
        
        # Normalize attitude angles
        self.x[6] = self._normalize_angle(self.x[6])  # phi
        self.x[7] = self._normalize_angle(self.x[7])  # theta
        self.x[8] = self._normalize_angle(self.x[8])  # psi
        
        # Predict covariance
        F = self._compute_jacobian_F(
            N, E, D, u, v, w, phi, theta, psi,
            f_measured, omega_measured
        )
        self.P = F @ self.P @ F.T + self.Q
    
    def update_imu(self, imu_accel: Dict, imu_gyro: Dict):
        """
        Update with IMU measurements.
        
        Args:
            imu_accel: {fx, fy, fz} [m/s²]
            imu_gyro: {p, q, r} [rad/s]
        """
        if not self.initialized:
            return
        
        # Store measurements for prediction
        self.last_imu_accel = np.array([
            imu_accel["fx"],
            imu_accel["fy"],
            imu_accel["fz"]
        ])
        self.last_imu_gyro = np.array([
            imu_gyro["p"],
            imu_gyro["q"],
            imu_gyro["r"]
        ])
        
        # Measurement vector
        z = np.concatenate([self.last_imu_accel, self.last_imu_gyro])
        
        # Measurement model (h(x))
        h = self._measurement_model_imu()
        
        # Measurement Jacobian
        H = self._compute_jacobian_H_imu()
        
        # Innovation
        y = z - h
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_imu
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.x += K @ y
        
        # Normalize attitude
        self.x[6] = self._normalize_angle(self.x[6])
        self.x[7] = self._normalize_angle(self.x[7])
        self.x[8] = self._normalize_angle(self.x[8])
        
        # Update covariance
        self.P = (np.eye(self.nx) - K @ H) @ self.P
    
    def update_gps(self, gps: Dict):
        """
        Update with GPS measurements.
        
        Args:
            gps: {lat_deg, lon_deg, alt_m, vn_mps, ve_mps, vd_mps}
        """
        if not self.initialized:
            return
        
        # Convert GPS position to NED (simplified)
        lat = gps["lat_deg"]
        lon = gps["lon_deg"]
        alt = gps["alt_m"]
        
        # Convert to NED (approximate)
        dlat = (lat - self.lat0_deg) * self.deg_to_rad
        dlon = (lon - self.lon0_deg) * self.deg_to_rad
        
        N_gps = dlat * self.R_earth
        E_gps = dlon * self.R_earth * math.cos(self.lat0_deg * self.deg_to_rad)
        D_gps = -(alt - (self.x[2] + self.lat0_deg * 0.0))  # Simplified
        
        # Measurement vector
        z = np.array([
            N_gps,
            E_gps,
            D_gps,
            gps["vn_mps"],
            gps["ve_mps"],
            gps["vd_mps"]
        ])
        
        # Measurement model
        V_ned = self._body_to_ned_velocity()
        h = np.concatenate([
            [self.x[0], self.x[1], self.x[2]],  # Position (N, E, D)
            V_ned  # Velocity in NED (3 elements)
        ])
        
        # Measurement Jacobian
        H = self._compute_jacobian_H_gps()
        
        # Innovation
        y = z - h
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_gps
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.x += K @ y
        
        # Normalize attitude
        self.x[6] = self._normalize_angle(self.x[6])
        self.x[7] = self._normalize_angle(self.x[7])
        self.x[8] = self._normalize_angle(self.x[8])
        
        # Update covariance
        self.P = (np.eye(self.nx) - K @ H) @ self.P
    
    def update_air_data(self, air_data: Dict):
        """
        Update with air data measurements.
        
        Args:
            air_data: {airspeed_mps, alpha_deg, beta_deg}
        """
        if not self.initialized:
            return
        
        # Measurement vector
        z = np.array([
            air_data["airspeed_mps"],
            math.radians(air_data["alpha_deg"]),
            math.radians(air_data["beta_deg"])
        ])
        
        # Measurement model (compute from state)
        u, v, w = self.x[3], self.x[4], self.x[5]
        V = math.sqrt(u**2 + v**2 + w**2)
        alpha = math.atan2(w, u) if V > 1e-3 else 0.0
        beta = math.asin(v / V) if V > 1e-3 else 0.0
        
        h = np.array([V, alpha, beta])
        
        # Measurement Jacobian
        H = self._compute_jacobian_H_air()
        
        # Innovation
        y = z - h
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_air
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.x += K @ y
        
        # Normalize attitude
        self.x[6] = self._normalize_angle(self.x[6])
        self.x[7] = self._normalize_angle(self.x[7])
        self.x[8] = self._normalize_angle(self.x[8])
        
        # Update covariance
        self.P = (np.eye(self.nx) - K @ H) @ self.P
    
    def update_magnetometer(self, magnetometer: Dict):
        """
        Update with magnetometer heading measurement.
        
        Args:
            magnetometer: {heading_deg}
        """
        if not self.initialized:
            return
        
        # Measurement
        z = math.radians(magnetometer["heading_deg"])
        
        # Measurement model (heading = psi)
        h = self.x[8]  # psi
        
        # Normalize innovation
        y = self._normalize_angle(z - h)
        
        # Measurement Jacobian
        H = np.zeros((1, self.nx))
        H[0, 8] = 1.0  # d(psi)/d(psi) = 1
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_mag
        
        # Kalman gain (1D for scalar measurement)
        K = (self.P @ H.T @ np.linalg.inv(S)).flatten()
        
        # Update state
        self.x += K * y
        
        # Normalize attitude
        self.x[8] = self._normalize_angle(self.x[8])
        
        # Update covariance
        self.P = (np.eye(self.nx) - np.outer(K, H.flatten())) @ self.P
    
    def update_baro(self, baro: Dict):
        """
        Update with barometric altimeter.
        
        Args:
            baro: {alt_m}
        """
        if not self.initialized:
            return
        
        # Measurement
        z = baro["alt_m"]
        
        # Measurement model (altitude = -D)
        h = -self.x[2]  # D is down, altitude is up
        
        # Innovation
        y = z - h
        
        # Measurement Jacobian
        H = np.zeros((1, self.nx))
        H[0, 2] = -1.0  # d(alt)/d(D) = -1
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_baro
        
        # Kalman gain (1D for scalar measurement)
        K = (self.P @ H.T @ np.linalg.inv(S)).flatten()
        
        # Update state
        self.x += K * y
        
        # Update covariance
        self.P = (np.eye(self.nx) - np.outer(K, H.flatten())) @ self.P
    
    def get_estimated_state(self) -> Dict:
        """
        Get estimated state as dictionary.
        
        Returns:
            State dictionary compatible with simulation
        """
        return {
            "N": self.x[0],
            "E": self.x[1],
            "D": self.x[2],
            "u": self.x[3],
            "v": self.x[4],
            "w": self.x[5],
            "phi": self.x[6],
            "theta": self.x[7],
            "psi": self.x[8],
            "h": -self.x[2],  # altitude
            # Computed values
            "V": math.sqrt(self.x[3]**2 + self.x[4]**2 + self.x[5]**2),
            "ba": self.x[9:12].copy(),  # accel bias
            "bg": self.x[12:15].copy()   # gyro bias
        }
    
    def _state_derivative(self, N, E, D, u, v, w, phi, theta, psi, f, omega):
        """Compute state derivative."""
        x_dot = np.zeros(self.nx)
        
        # Position derivative (NED velocity from body)
        R_bn = dcm_body_to_ned(phi, theta, psi)
        V_body = np.array([u, v, w])
        V_ned = R_bn @ V_body
        x_dot[0] = V_ned[0]  # N_dot
        x_dot[1] = V_ned[1]  # E_dot
        x_dot[2] = V_ned[2]  # D_dot
        
        # Velocity derivative (from accelerometer)
        # f = a_body - g_body, so a_body = f + g_body
        g_ned = np.array([0.0, 0.0, self.g])
        g_body = R_bn.T @ g_ned
        a_body = f + g_body
        
        # Add centrifugal terms: omega x V
        omega_vec = omega
        V_body_vec = np.array([u, v, w])
        centrifugal = np.cross(omega_vec, V_body_vec)
        a_body += centrifugal
        
        x_dot[3] = a_body[0]  # u_dot
        x_dot[4] = a_body[1]  # v_dot
        x_dot[5] = a_body[2]  # w_dot
        
        # Attitude derivative (Euler kinematics)
        p, q, r = omega[0], omega[1], omega[2]
        x_dot[6] = p + math.tan(theta) * (q * math.sin(phi) + r * math.cos(phi))  # phi_dot
        x_dot[7] = q * math.cos(phi) - r * math.sin(phi)  # theta_dot
        x_dot[8] = (q * math.sin(phi) + r * math.cos(phi)) / math.cos(theta)  # psi_dot
        
        # Bias derivative (random walk, zero in prediction)
        x_dot[9:15] = np.zeros(6, dtype=np.float64)
        
        return x_dot
    
    def _measurement_model_imu(self) -> np.ndarray:
        """IMU measurement model."""
        u, v, w = self.x[3], self.x[4], self.x[5]
        phi, theta, psi = self.x[6], self.x[7], self.x[8]
        ba = self.x[9:12]
        bg = self.x[12:15]
        
        # Accelerometer measurement (specific force + bias)
        R_bn = dcm_body_to_ned(phi, theta, psi)
        g_ned = np.array([0.0, 0.0, self.g])
        g_body = R_bn.T @ g_ned
        
        # Approximate acceleration from kinematics
        omega = self.last_imu_gyro - bg
        V_body = np.array([u, v, w])
        centrifugal = np.cross(omega, V_body)
        
        # Specific force = a_body - g_body
        f_expected = -g_body - centrifugal + ba
        
        # Gyroscope measurement (angular rates + bias)
        omega_expected = omega + bg
        
        return np.concatenate([f_expected, omega_expected])
    
    def _body_to_ned_velocity(self) -> np.ndarray:
        """Convert body velocity to NED velocity."""
        R_bn = dcm_body_to_ned(self.x[6], self.x[7], self.x[8])
        V_body = np.array([self.x[3], self.x[4], self.x[5]])
        return R_bn @ V_body
    
    def _compute_jacobian_F(self, N, E, D, u, v, w, phi, theta, psi, f, omega):
        """Compute state transition Jacobian (simplified)."""
        # Simplified: identity + small corrections
        F = np.eye(self.nx)
        
        # Position derivatives
        R_bn = dcm_body_to_ned(phi, theta, psi)
        dR_dphi, dR_dtheta, dR_dpsi = self._dcm_derivatives(phi, theta, psi)
        
        V_body = np.array([u, v, w])
        
        # d(N_dot)/d(phi, theta, psi) - ensure scalar assignment
        try:
            F[0, 6] = float((dR_dphi[0, :] @ V_body) * self.dt)
            F[0, 7] = float((dR_dtheta[0, :] @ V_body) * self.dt)
            F[0, 8] = float((dR_dpsi[0, :] @ V_body) * self.dt)
            
            F[1, 6] = float((dR_dphi[1, :] @ V_body) * self.dt)
            F[1, 7] = float((dR_dtheta[1, :] @ V_body) * self.dt)
            F[1, 8] = float((dR_dpsi[1, :] @ V_body) * self.dt)
            
            F[2, 6] = float((dR_dphi[2, :] @ V_body) * self.dt)
            F[2, 7] = float((dR_dtheta[2, :] @ V_body) * self.dt)
            F[2, 8] = float((dR_dpsi[2, :] @ V_body) * self.dt)
        except:
            # Fallback: skip if computation fails
            pass
        
        # Velocity derivatives (simplified - skip complex derivatives for now)
        # F[3, 6:9] = self._daccel_dattitude(phi, theta, psi, f) * self.dt
        # F[4, 6:9] = self._daccel_dattitude(phi, theta, psi, f) * self.dt
        # F[5, 6:9] = self._daccel_dattitude(phi, theta, psi, f) * self.dt
        # Simplified: assume small coupling (will refine later)
        
        # Attitude derivatives (from gyro)
        F[6, 6] = 1.0 + self.dt * math.tan(theta) * (omega[1] * math.cos(phi) - omega[2] * math.sin(phi))
        F[6, 7] = self.dt * (omega[1] * math.sin(phi) + omega[2] * math.cos(phi)) / (math.cos(theta)**2)
        
        F[7, 6] = -self.dt * (omega[1] * math.sin(phi) + omega[2] * math.cos(phi))
        F[7, 7] = 1.0
        
        F[8, 6] = self.dt * (omega[1] * math.cos(phi) - omega[2] * math.sin(phi)) / math.cos(theta)
        F[8, 7] = self.dt * (omega[1] * math.sin(phi) + omega[2] * math.cos(phi)) * math.tan(theta) / math.cos(theta)
        
        return F
    
    def _compute_jacobian_H_imu(self) -> np.ndarray:
        """IMU measurement Jacobian."""
        H = np.zeros((6, self.nx))
        
        # Accelerometer
        phi, theta, psi = self.x[6], self.x[7], self.x[8]
        dR_dphi, dR_dtheta, dR_dpsi = self._dcm_derivatives(phi, theta, psi)
        g_ned = np.array([0.0, 0.0, self.g])
        
        # d(f)/d(phi, theta, psi) - ensure 1D array assignment
        try:
            H[0:3, 6] = (-dR_dphi.T @ g_ned).flatten()
            H[0:3, 7] = (-dR_dtheta.T @ g_ned).flatten()
            H[0:3, 8] = (-dR_dpsi.T @ g_ned).flatten()
        except:
            # Fallback: skip if computation fails
            pass
        
        # d(f)/d(bias)
        H[0:3, 9:12] = np.eye(3)
        
        # Gyroscope
        H[3:6, 12:15] = np.eye(3)  # d(omega)/d(bias)
        
        return H
    
    def _compute_jacobian_H_gps(self) -> np.ndarray:
        """GPS measurement Jacobian."""
        H = np.zeros((6, self.nx))
        
        # Position
        H[0, 0] = 1.0  # N
        H[1, 1] = 1.0  # E
        H[2, 2] = 1.0  # D
        
        # Velocity (NED from body)
        phi, theta, psi = self.x[6], self.x[7], self.x[8]
        R_bn = dcm_body_to_ned(phi, theta, psi)
        H[3:6, 3:6] = R_bn  # d(V_ned)/d(V_body)
        
        # d(V_ned)/d(attitude)
        dR_dphi, dR_dtheta, dR_dpsi = self._dcm_derivatives(phi, theta, psi)
        V_body = np.array([self.x[3], self.x[4], self.x[5]])
        try:
            H[3:6, 6] = (dR_dphi @ V_body).flatten()
            H[3:6, 7] = (dR_dtheta @ V_body).flatten()
            H[3:6, 8] = (dR_dpsi @ V_body).flatten()
        except:
            # Fallback: skip if computation fails
            pass
        
        return H
    
    def _compute_jacobian_H_air(self) -> np.ndarray:
        """Air data measurement Jacobian."""
        H = np.zeros((3, self.nx))
        
        u, v, w = self.x[3], self.x[4], self.x[5]
        V = math.sqrt(u**2 + v**2 + w**2)
        
        if V > 1e-3:
            # d(V)/d(u, v, w)
            H[0, 3] = u / V
            H[0, 4] = v / V
            H[0, 5] = w / V
            
            # d(alpha)/d(u, w)
            H[1, 3] = -w / (u**2 + w**2)
            H[1, 5] = u / (u**2 + w**2)
            
            # d(beta)/d(v, V)
            H[2, 4] = 1.0 / math.sqrt(V**2 - v**2)
            H[2, 3] = -v * u / (V**2 * math.sqrt(V**2 - v**2))
            H[2, 5] = -v * w / (V**2 * math.sqrt(V**2 - v**2))
        
        return H
    
    def _dcm_derivatives(self, phi, theta, psi):
        """Compute DCM derivatives w.r.t. Euler angles."""
        # Simplified derivatives (approximate)
        # Full implementation would compute exact derivatives of rotation matrix
        # For now, use small perturbation approximation
        eps = 1e-6
        try:
            R0 = dcm_body_to_ned(phi, theta, psi)
            
            dR_dphi = (dcm_body_to_ned(phi + eps, theta, psi) - R0) / eps
            dR_dtheta = (dcm_body_to_ned(phi, theta + eps, psi) - R0) / eps
            dR_dpsi = (dcm_body_to_ned(phi, theta, psi + eps) - R0) / eps
            
            # Ensure proper array types
            dR_dphi = np.asarray(dR_dphi, dtype=np.float64)
            dR_dtheta = np.asarray(dR_dtheta, dtype=np.float64)
            dR_dpsi = np.asarray(dR_dpsi, dtype=np.float64)
        except:
            # Fallback: return zero matrices if computation fails
            dR_dphi = np.zeros((3, 3), dtype=np.float64)
            dR_dtheta = np.zeros((3, 3), dtype=np.float64)
            dR_dpsi = np.zeros((3, 3), dtype=np.float64)
        
        return dR_dphi, dR_dtheta, dR_dpsi
    
    def _daccel_dattitude(self, phi, theta, psi, f):
        """Derivative of acceleration w.r.t. attitude."""
        # Simplified: return zeros for now (will refine later)
        return np.zeros(3, dtype=np.float64)
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

