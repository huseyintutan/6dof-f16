# f16_constants.py
# -------------------------------------------------------------
# F-16 Fighting Falcon - Fixed Physical and Geometric Parameters
# This file defines constant values used in the simulation.
# No functions are defined here – only numerical constants.
# -------------------------------------------------------------

F16_CONSTANTS = {
    # =========================================================
    # GEOMETRY PARAMETERS
    # =========================================================
    "S": 27.87,        # [m^2] Wing reference area
    "b": 9.144,        # [m]   Wing span
    "cbar": 3.45,      # [m]   Mean aerodynamic chord

    # =========================================================
    # MASS AND INERTIA PROPERTIES
    # (Components of the inertia tensor in body axes)
    # =========================================================
    "mass": 9300.0,    # [kg] Aircraft total mass (empty + pilot + fuel)
    "Ixx": 12875.0,    # [kg·m²] Roll axis moment of inertia (x-axis)
    "Iyy": 75674.0,    # [kg·m²] Pitch axis moment of inertia (y-axis)
    "Izz": 85552.0,    # [kg·m²] Yaw axis moment of inertia (z-axis)
    "Ixz": 1331.0,     # [kg·m²] Cross inertia term (x–z coupling)
    "Ixy": 0.0,        # [kg·m²] Usually zero (assumed symmetry)
    "Iyz": 0.0,        # [kg·m²] Usually zero (assumed symmetry)

    # =========================================================
    # ATMOSPHERIC AND GRAVITATIONAL CONSTANTS
    # =========================================================
    "g": 9.80665,      # [m/s²] Gravitational acceleration (ISA standard)
    "R": 287.05287,    # [J/(kg·K)] Gas constant for air
    "gamma": 1.4,      # [-] Specific heat ratio for air
    "rho0": 1.225,     # [kg/m³] Sea level density
    "T0": 288.15,      # [K] Sea level temperature (15°C)
    "p0": 101325.0,    # [Pa] Sea level pressure

    # =========================================================
    # REFERENCE TRIM CONDITION
    # =========================================================
    "alpha_trim": 2.0, # [deg] Trim angle of attack (approx. level flight)
    "V_trim": 150.0,   # [m/s] Trim velocity (~300 knots)
    "h_trim": 3048.0,  # [m]   Trim altitude (≈10,000 ft)

    # =========================================================
    # CONTROL SURFACE LIMITS (for controller or actuator modeling)
    # =========================================================
    "de_max": 25.0,    # [deg] Elevator max deflection
    "da_max": 21.5,    # [deg] Aileron max deflection
    "dr_max": 30.0,    # [deg] Rudder max deflection
    "lef_max": 25.0,   # [deg] Leading-edge flap max deflection

    # =========================================================
    # ENGINE AND THRUST DATA (approximate values)
    # =========================================================
    "Tmax_SL": 129000.0,   # [N] Max thrust at sea level (~29,000 lbf)
    "Tmax_alt": 92000.0,   # [N] Max thrust at ~10,000 ft (approx.)
}

# -------------------------------------------------------------
# NOTES:
# - Inertia data (Ixx, Iyy, Izz, Ixz) are taken from USAF/NASA F-16 documentation.
# - "Ixz" represents roll–yaw coupling due to asymmetry.
# - These constants are valid for clean configuration, medium fuel load.
# - Engine thrust decreases with altitude (Tmax_alt is approximate).
# -------------------------------------------------------------
