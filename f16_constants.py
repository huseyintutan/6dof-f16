# f16_constants.py
# Minimal constant pack used by dynamics & trim

import math

# Geometry / mass (senin verdiğin değerlere göre)
I_xx = 12875.0     # kg m^2
I_yy = 75674.0
I_zz = 85552.0
I_xz = 1331.0
I_yx = 0.0
I_yz = 0.0

MASS = 9300.0      # kg
S     = 27.88      # m^2
B     = 9.144      # m (span)
C_BAR = 3.45       # m (mean aero chord)

# Atmosphere / gravity
G0 = 9.80665       # m/s^2

# Trim targets
V_TRIM = 150.0     # m/s (isteğe göre 153 de olabilir)
H_TRIM = 3048.0    # m

# Engine (yaklaşık; ihtiyaca göre güncelleyebiliriz)
TMAX_ALT = 120_000.0  # N (AB yakın maksimum; kabaca)

F16_CONSTANTS = {
    # Inertia as 3x3 (body frame; Ixy=Iyz=0, Ixz ≠ 0)
    "I": [
        [ I_xx,      -I_yx,     -I_xz ],
        [ -I_yx,     I_yy,      -I_yz ],
        [ -I_xz,     -I_yz,      I_zz ],
    ],

    # Also expose components individually (fallback için)
    "I_xx": I_xx, "I_yy": I_yy, "I_zz": I_zz,
    "I_xz": I_xz, "I_yx": I_yx, "I_yz": I_yz,

    "mass": MASS,
    "S": S,
    "b": B,
    "c_bar": C_BAR,

    "g": G0,

    "V_trim": V_TRIM,
    "h_trim": H_TRIM,

    "Tmax_alt": TMAX_ALT,
}
