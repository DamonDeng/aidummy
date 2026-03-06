# Joint Direction Verification Guide

## Date: 2026-03-06

---

## 1. Coordinate System (from FK Analysis)

At L-Pose `{0, 0, 90, 0, 0, 0}` with J1=0, the FK computes end effector at `(X=222mm, Y=0, Z=307mm)`.

- **+X** = robot's forward direction (when J1=0)
- **+Y** = robot's left
- **+Z** = up

DH parameters (from `6dof_kinematic.cpp`):
```
DOF6Kinematic(0.109f, 0.035f, 0.146f, 0.115f, 0.052f, 0.072f)
  L_BASE=109mm, D_BASE=35mm, L_ARM=146mm, L_FOREARM=115mm, D_ELBOW=52mm, L_WRIST=72mm
```

---

## 2. Positive Joint Angle Direction (from DH/FK/IK)

### J1 — Base Rotation
- **Axis:** Z0 (vertical, pointing up)
- **Positive direction:** Counterclockwise from above = turning LEFT
- **Source:** IK formula `qs[0] = atan2(Py, Px)` — J1=0 faces +X (forward), J1=+90° faces +Y (left)

### J2 — Shoulder (Lower Arm) — ALREADY CONFIRMED
- **Positive direction:** Unfold (arm moves forward/up from REST toward L-Pose)
- J2 goes from -73° (REST) to 0° (L-Pose), so positive = unfolding

### J3 — Elbow (Upper Arm) — ALREADY CONFIRMED
- **Positive direction:** Fold (toward 180°)
- J3 goes from 90° (L-Pose) to 180° (REST), so positive = folding closed

### J4 — Wrist Roll (around forearm axis)
- **Axis at L-Pose:** +X (forward, along the arm)
- **Positive direction:** CCW when looking from shoulder toward hand (right-hand rule around +X)

### J5 — Wrist Pitch (tilt up/down)
- **Axis at L-Pose:** +Y (horizontal, perpendicular to arm, pointing left)
- **Positive direction:** End effector tilts DOWNWARD (right-hand rule around +Y)

### J6 — End Effector Roll (around forearm axis)
- **Axis at L-Pose:** +X (same as J4)
- **Positive direction:** CCW when looking from shoulder toward hand (same as J4)

---

## 3. Current `inverseDirection` Settings

```cpp
motorJ[1] = new CtrlStepMotor(_hcan, 1, true,  30, -170, 170);  // J1
motorJ[2] = new CtrlStepMotor(_hcan, 2, false, 30, -73, 90);    // J2 — CONFIRMED CORRECT
motorJ[3] = new CtrlStepMotor(_hcan, 3, false, 30, 35, 180);    // J3 — CONFIRMED CORRECT
motorJ[4] = new CtrlStepMotor(_hcan, 4, false, 24, -180, 180);  // J4
motorJ[5] = new CtrlStepMotor(_hcan, 5, true,  30, -120, 120);  // J5
motorJ[6] = new CtrlStepMotor(_hcan, 6, true,  50, -720, 720);  // J6
```

---

## 4. How `inverseDirection` Translates MoveJ to Raw Motor

For MoveJ with +10° increase on a single joint (from REST, initPose=REST_POSE):

```
motor_angle = target - initPose = +10°
If inverseDirection=true:  raw_turns = -10 / 360 * reduction
If inverseDirection=false: raw_turns = +10 / 360 * reduction
```

| Joint | inverseDirection | reduction | MoveJ +10° → raw turns | Raw must physically = |
|-------|-----------------|-----------|------------------------|----------------------|
| J1 | true | 30 | **-0.833** | Turn LEFT (CCW from above) |
| J4 | false | 24 | **+0.667** | Roll CCW (shoulder→hand view) |
| J5 | true | 30 | **-0.833** | Tilt DOWN |
| J6 | true | 50 | **-1.389** | Roll CCW (shoulder→hand view) |

---

## 5. Test Procedure

### Step 1: Test raw motor direction

From REST position (all motor encoders at 0), test each joint one at a time:

```python
# J1 — observe: does base turn LEFT or RIGHT?
dummy0.robot.joint_1.set_position_with_time(1, 2)
dummy0.robot.joint_1.set_position_with_time(0, 2)

# J4 — observe: does wrist roll CCW or CW (looking from shoulder toward hand)?
dummy0.robot.joint_4.set_position_with_time(1, 2)
dummy0.robot.joint_4.set_position_with_time(0, 2)

# J5 — observe: does end effector tilt UP or DOWN?
dummy0.robot.joint_5.set_position_with_time(1, 2)
dummy0.robot.joint_5.set_position_with_time(0, 2)

# J6 — observe: does wrist roll CCW or CW (looking from shoulder toward hand)?
dummy0.robot.joint_6.set_position_with_time(1, 2)
dummy0.robot.joint_6.set_position_with_time(0, 2)
```

Note: `set_position_with_time()` is a raw motor command — `inverseDirection` does NOT apply.

### Step 2: Determine correct flag using decision tables

**J1** (currently `inverseDirection=true`):

| Raw +1 physically = | inverseDirection should be | Current `true` is |
|---------------------|--------------------------|-------------------|
| Turns LEFT (CCW from above) | `false` | **WRONG** |
| Turns RIGHT (CW from above) | `true` | **CORRECT** |

**J4** (currently `inverseDirection=false`):

| Raw +1 physically = | inverseDirection should be | Current `false` is |
|---------------------|--------------------------|-------------------|
| Rolls CCW (shoulder→hand) | `false` | **CORRECT** |
| Rolls CW (shoulder→hand) | `true` | **WRONG** |

**J5** (currently `inverseDirection=true`):

| Raw +1 physically = | inverseDirection should be | Current `true` is |
|---------------------|--------------------------|-------------------|
| Tilts DOWN | `false` | **WRONG** |
| Tilts UP | `true` | **CORRECT** |

**J6** (currently `inverseDirection=true`):

| Raw +1 physically = | inverseDirection should be | Current `true` is |
|---------------------|--------------------------|-------------------|
| Rolls CCW (shoulder→hand) | `true` | **CORRECT** |
| Rolls CW (shoulder→hand) | `false` | **WRONG** |

---

## 6. Important Notes

- J1, J4, J5, J6 all stay at 0° during `Homing()` and `calibrate_home_offset()`, so wrong flags on these joints won't affect those critical operations. They only matter for `MoveJ()` and `MoveL()`.
- The wrist directions (J4, J5, J6) are described at L-Pose where the arm extends forward horizontally. At REST (arm folded), the same motor rotation looks different in world coordinates because the arm geometry has changed, but the local joint behavior is the same.
- J5 "tilt DOWN" at L-Pose means the end effector dips below the horizontal. At REST (folded), this same motion may look like it moves in a different world-frame direction because the forearm is pointed differently.

---

## 7. Test Results

| Joint | Raw +1 direction | inverseDirection should be | Current setting | Status |
|-------|-----------------|--------------------------|-----------------|--------|
| J1 | CW from above (RIGHT) | `true` | `true` | **CONFIRMED CORRECT** |
| J2 | Moves FORWARD | `false` | `false` | **CONFIRMED CORRECT** |
| J3 | Folds (toward 180°+) | `false` | `false` | **CONFIRMED CORRECT** |
| J4 | CW (shoulder→hand view) | `true` | `false` | **WRONG — needs change** |
| J5 | Tilts DOWN | `false` | `true` | **WRONG — needs change** |
| J6 | CCW (shoulder→hand view) | `false` | `true` | **WRONG — needs change** |

### All Issues Fixed (2026-03-06)

Final firmware values (all verified and tested):
```cpp
motorJ[1] = new CtrlStepMotor(_hcan, 1, true,  50, -170, 170);
motorJ[2] = new CtrlStepMotor(_hcan, 2, false, 50, -73, 90);
motorJ[3] = new CtrlStepMotor(_hcan, 3, false, 50, 35, 180);
motorJ[4] = new CtrlStepMotor(_hcan, 4, true,  40, -180, 180);
motorJ[5] = new CtrlStepMotor(_hcan, 5, false, 50, -120, 120);
motorJ[6] = new CtrlStepMotor(_hcan, 6, false, 50, -720, 720);
```

- `Homing()` reaches proper L-Pose (lower arm vertical, upper arm horizontal) ✓
- `Resting()` folds back to REST position ✓
- Home offset set via manual approach (`apply_home_offset()` at REST) ✓

### Test Details (2026-03-06)

**J1 — Base Rotation:**
- `set_position_with_time(2, 2)` → base rotates CW from above (RIGHT)
- `set_position_with_time(-2, 2)` → base rotates CCW from above (LEFT)
- MoveJ(+10°) needs CCW (LEFT) → sends raw -0.833 → raw negative = LEFT ✓
- `inverseDirection=true` is **CORRECT**

**J4 — Wrist Roll (tested at L-Pose):**
- `set_position_with_time(3, 2)` → CW looking from shoulder to hand
- `set_position_with_time(-3, 2)` → CCW looking from shoulder to hand
- MoveJ(+10°) needs CCW → with `inverseDirection=false` sends raw +0.667 → raw positive = CW ✗
- `inverseDirection=false` is **WRONG**, should be `true`

**J5 — Wrist Pitch (tested at L-Pose, J4 at 0):**
- `set_position_with_time(4, 2)` → hand points DOWN
- `set_position_with_time(-4, 2)` → hand points UP
- MoveJ(+10°) needs DOWN → with `inverseDirection=true` sends raw -0.833 → raw negative = UP ✗
- `inverseDirection=true` is **WRONG**, should be `false`

**J6 — End Effector Roll (tested at L-Pose, J4 and J5 at 0):**
- `set_position_with_time(3, 2)` → CCW looking from shoulder to hand
- `set_position_with_time(-3, 2)` → CW looking from shoulder to hand
- MoveJ(+10°) needs CCW → with `inverseDirection=true` sends raw -1.389 → raw negative = CW ✗
- `inverseDirection=true` is **WRONG**, should be `false`

### Summary

4 out of 6 joints differ from the reference design. Only J1 and J2 match:

| Joint | Reference design | Our hardware | Match? |
|-------|-----------------|-------------|--------|
| J1 | `true` | `true` | Yes |
| J2 | `false` | `false` | Yes |
| J3 | `true` | `false` | **No** |
| J4 | `false` | `true` | **No** |
| J5 | `true` | `false` | **No** |
| J6 | `true` | `false` | **No** |

---

## 8. Reduction Ratio Findings (2026-03-06)

### Problem
After Homing(), the arm formed a Z shape instead of L-Pose. The lower arm was not vertical while the upper arm appeared horizontal. This was caused by **incorrect reduction ratios** in the firmware, not imprecise manual positioning.

### Root Cause
The user's reducers are all **50:1**, but the reference design used different ratios. Every movement was scaled incorrectly — joints moved less than intended (undershoot).

### Reference Design vs Our Hardware

| Joint | Motor type | Reducer type | Reference reduction | Our actual reduction | Scale error |
|-------|-----------|-------------|--------------------|--------------------|-------------|
| J1 | 42-motor | 42-reducer | 30 | **50** | 60% of intended |
| J2 | 42-motor | 42-reducer | 30 | **50** | 60% of intended |
| J3 | 42-motor | 42-reducer | 30 | **50** | 60% of intended |
| J4 | 35-motor | 42-reducer | 24 | **40** | 48% of intended |
| J5 | 35-motor | 35-reducer | 30 | **50** | 60% of intended |
| J6 | 35-motor | 35-reducer | 50 | **50** | Correct! |

### J4 Special Case — Belt/Pulley Coupling
J4 uses a 35-motor with a 42-motor reducer. There's a belt/pulley mechanical coupling between the motor and the gearbox input that changes the effective ratio:

```
effective_reduction = gearbox_ratio × coupling_ratio

Original:  24 = 30 × coupling_ratio → coupling_ratio = 0.8
Our setup: effective = 50 × 0.8 = 40
```

The coupling ratio (0.8, likely a 4:5 tooth pulley pair) is a property of the mechanical design, same between reference and our build. Only the gearbox ratio changed (30→50).

### Why the Z Shape Explained
During Homing from REST to L-Pose with reduction=30 (code) but actual=50:
- J2: intended 73° movement, actual = 73 × 30/50 = **43.8°** → lower arm only moved 60%, didn't reach vertical
- J3: intended 90° movement, actual = 90 × 30/50 = **54°** → upper arm only moved 60%, didn't reach horizontal

Both joints undershot by the same ratio. The lower arm being "not vertical" was much more visually obvious than the upper arm being "not quite horizontal", which is why the upper arm appeared correct.

### Impact
The reduction ratio affects ALL high-level movements (MoveJ, MoveL, Homing, Resting, calibrate_home_offset) and angle reporting. It does NOT affect raw motor commands (set_position_with_time, set_position).

The fix is REF board firmware only — motor driver boards don't know about the reduction ratio.

---

### Root Cause Analysis (IMPORTANT)

The original theory (motor coil wiring / encoder magnet orientation) explains J4-J6 (all 35-motors, user-welded, all consistently wrong). But it does NOT explain J3 (42-motor), because:

- J1, J2, J3 are all 42-motors with **identical coil wiring** (user welded all the same way)
- J1 and J2 driver boards were **flashed by the motor vendor** → direction matches reference
- J3 driver board was **reflashed by us** (self-built firmware from repo) → direction is opposite

**The difference is the motor driver firmware, not the hardware wiring.**

During `do_calibration()`, the `goDirection` flag is determined by which direction encoder counts increase when the firmware drives the motor forward. Our self-built 42-motor firmware (`dummy-42motor-fw`) produces the **opposite `goDirection`** compared to the vendor's pre-flashed firmware, for the same physical hardware.

This means:
1. **If J1 or J2 ever need reflashing** with our self-built firmware → after `do_calibration()`, their `goDirection` will likely flip → their `inverseDirection` must be changed from `true` to `false` (same as J3)
2. **For J4-J6** (35-motors): we don't have vendor firmware to compare, but the consistent wrongness suggests either the 35-motor firmware also has this difference, or the coil wiring is consistently opposite
3. **The "correct" `inverseDirection` depends on which firmware is on the motor driver board**, not just the physical hardware

This is a critical coupling between the REF board firmware (`inverseDirection` flags) and the motor driver firmware version. They must be kept in sync.
