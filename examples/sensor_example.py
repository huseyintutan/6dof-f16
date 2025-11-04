# sensor_example.py
# -------------------------------------------------------------
# Example usage of F-16 sensor suite
# 
# This script demonstrates how to use the sensor models
# independently or integrated into a simulation.
# -------------------------------------------------------------

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from src.f16_sensors import SensorSuite
import math

# Example: Create sensor suite and simulate measurements
if __name__ == "__main__":
    print("=== F-16 Sensor Suite Example ===\n")
    
    # Initialize sensors with noise and bias enabled
    sensors = SensorSuite(
        enable_noise=True,
        enable_bias=True,
        seed=42
    )
    
    # Example state (trimmed level flight)
    state = {
        "u": 150.0,      # m/s - forward velocity
        "v": 0.0,        # m/s - lateral velocity
        "w": 5.0,        # m/s - vertical velocity
        "p": 0.0,        # rad/s - roll rate
        "q": 0.0,        # rad/s - pitch rate
        "r": 0.0,        # rad/s - yaw rate
        "phi": 0.0,      # rad - roll angle
        "theta": 0.03,   # rad - pitch angle (~1.7 deg)
        "psi": 0.0,      # rad - yaw angle
        "h": 3048.0      # m - altitude
    }
    
    # NED position (relative to reference point)
    N, E, D = 0.0, 0.0, -3048.0
    lat0_deg = 41.015137  # Istanbul
    lon0_deg = 28.979530
    
    # Simulate for a few time steps
    dt = 0.01  # 10 ms
    t_total = 1.0  # 1 second
    
    print("Simulating sensors for 1 second (100 time steps)...\n")
    
    for i in range(int(t_total / dt)):
        t = i * dt
        
        # Update sensors
        measurements = sensors.update(
            t, state, N, E, D, lat0_deg, lon0_deg, dt
        )
        
        # Print every 0.1 seconds
        if i % 10 == 0:
            print(f"t = {t:.2f} s")
            print(f"  IMU Accelerometer: fx={measurements['imu_accel']['fx']:.3f}, "
                  f"fy={measurements['imu_accel']['fy']:.3f}, "
                  f"fz={measurements['imu_accel']['fz']:.3f} m/s²")
            print(f"  IMU Gyroscope: p={measurements['imu_gyro']['p']:.4f}, "
                  f"q={measurements['imu_gyro']['q']:.4f}, "
                  f"r={measurements['imu_gyro']['r']:.4f} rad/s")
            print(f"  GPS: lat={measurements['gps']['lat_deg']:.6f}°, "
                  f"lon={measurements['gps']['lon_deg']:.6f}°, "
                  f"alt={measurements['gps']['alt_m']:.1f} m")
            print(f"  Air Data: V={measurements['air_data']['airspeed_mps']:.2f} m/s, "
                  f"α={measurements['air_data']['alpha_deg']:.2f}°, "
                  f"β={measurements['air_data']['beta_deg']:.2f}°")
            print(f"  Magnetometer: heading={measurements['magnetometer']['heading_deg']:.2f}°")
            print(f"  Barometric Altimeter: h={measurements['baro_alt']['alt_m']:.1f} m")
            print()
    
    print("Sensor simulation complete!")
    print("\nNote: Sensor outputs include noise and bias as configured.")
    print("Compare with true values:")
    print(f"  True airspeed: {math.sqrt(state['u']**2 + state['v']**2 + state['w']**2):.2f} m/s")
    print(f"  True altitude: {state['h']:.1f} m")
    print(f"  True heading: {math.degrees(state['psi']):.2f}°")
