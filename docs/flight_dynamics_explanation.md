# Flight Dynamics: Speed Control Explanation

## Current Speed Control Mechanism

### Forces Affecting Speed

The aircraft's forward acceleration (`u_dot`) is determined by the net force in the X-body axis:

```
Fx = Thrust + Aerodynamic_X + Gravity_X
```

Where:
- **Thrust**: Engine force along +x_body (currently **FIXED** after trim)
- **Aerodynamic_X**: Drag and lift components projected onto x-axis
- **Gravity_X**: Gravity component in body frame

### Detailed Force Breakdown

#### 1. Aerodynamic Forces
```
X_aero = -0.5·ρ·V²·S·CD·cos(α) + 0.5·ρ·V²·S·CL·sin(α)
```

- **Drag**: `D = 0.5·ρ·V²·S·CD` (proportional to V²)
- **Lift**: `L = 0.5·ρ·V²·S·CL`
- **CD coefficient**: `CD = CD0 + k·CL²` (parabolic drag polar)
- **CD depends on**: α (angle of attack), Mach number

#### 2. Thrust (Currently Fixed)
- Calculated during trim to balance drag at trim speed
- **Remains constant** during maneuvers
- Value: `thrust_N` (from trim solution)

#### 3. Gravity Component
```
Gravity_X = m·g·sin(θ)  (in body frame)
```
- Depends on pitch angle (θ)
- Positive when climbing (pitch up)

### Speed Change Equation

```
u_dot = (Fx - Coriolis_terms) / m
```

Where:
- `Fx = Thrust - Drag·cos(α) + Lift·sin(α) + Gravity_X`
- If `Fx > 0`: aircraft accelerates
- If `Fx < 0`: aircraft decelerates

## Why Speed Changes During Maneuvers

### During Turn Maneuver

1. **Aileron input** → aircraft rolls
2. **Lift vector tilts** → vertical lift component decreases
3. **Aircraft banks** → needs more α to maintain altitude
4. **Increased α** → increased drag (`CD = CD0 + k·CL²`)
5. **Thrust remains constant** → drag > thrust
6. **Result**: Aircraft decelerates

### During Climb Maneuver

1. **Elevator input** → aircraft pitches up
2. **Increased α** → increased drag
3. **Gravity component** → `Gravity_X = m·g·sin(θ)` (opposes forward motion when climbing)
4. **Thrust remains constant** → drag + gravity > thrust
5. **Result**: Aircraft decelerates

### During Level Flight (No Maneuver)

1. **Trimmed state** → Thrust = Drag
2. **α = α_trim** → drag at minimum (for given lift)
3. **Result**: Speed remains constant (in ideal conditions)

## Current Limitations

### Problem
- **Thrust is fixed** after trim
- **Drag changes** during maneuvers (due to α, β, V changes)
- **Imbalance** → speed drifts away from trim speed

### Real Aircraft Behavior
Real aircraft have:
- **Autothrottle** (speed hold mode)
- **Pilot throttle input** (manual speed control)
- **Throttle response** to maintain desired speed

## Solutions

### Option 1: Fixed Thrust (Current)
- ✅ Simple
- ❌ Speed drifts during maneuvers
- ❌ Not realistic for long maneuvers

### Option 2: Speed Hold Autothrottle
- ✅ Maintains constant speed
- ✅ More realistic
- ⚠️ Requires throttle controller implementation

### Option 3: Manual Throttle Control
- ✅ Full pilot control
- ⚠️ Requires input interface (keyboard/joystick)

## Recommended: Add Speed Hold

A simple speed hold controller would:
1. Measure current speed (V)
2. Compare to target speed (V_target)
3. Adjust thrust proportionally:
   ```
   thrust_new = thrust_trim + Kp·(V_target - V)
   ```
4. Maintain speed during maneuvers

This would make the simulation more realistic and prevent speed drift.

