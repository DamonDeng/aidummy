# Calibration Troubleshooting - Lessons Learned

## Date: 2026-03-05

---

## 1. Understanding the Calibration System

### Two-Level Calibration
1. **Encoder calibration** (`do_calibration()`) — per motor, stored on motor driver board flash
2. **Home offset calibration** (`calibrate_home_offset()` or manual `apply_home_offset()`) — sets motor encoder to 0 at a known position, stored on motor driver board EEPROM

### "Home Position" Terminology (Confusing!)
There are **two different "home" concepts**:

| Term | Meaning | Value |
|------|---------|-------|
| Motor "home" (encoder = 0) | The position where motor encoder reads 0, set by `ApplyPositionAsHome()` | REST_POSE after calibration |
| Robot `Homing()` function | Moves the robot to L-Pose | `{0, 0, 90, 0, 0, 0}` |

The boot-up position is **REST_POSE** `{0, -73, 180, 0, 0, 0}` (motor encoders = 0).
`Homing()` moves from REST_POSE to **L-Pose** `{0, 0, 90, 0, 0, 0}`.

### How `calibrate_home_offset()` Works (Two-Step Process)
1. User positions arm at L-Pose → first `ApplyPositionAsHome()` (temporary reference)
2. Firmware moves arm to REST_POSE (verifies directions are correct)
3. Second `ApplyPositionAsHome()` at REST_POSE → sets the final boot-up reference
4. Reboots

**Alternative manual approach:** Position arm at REST_POSE → call `apply_home_offset()` on all joints → reboot. This skips the verification step but avoids direction issues during calibration.

### `initPose` — The Bridge
```cpp
DOF6Kinematic::Joint6D_t initPose = REST_POSE;  // {0, -73, 180, 0, 0, 0}
```
This tells the firmware: "when motor encoders read 0, the robot joints are at these angles."

---

## 2. Understanding `inverseDirection`

### What It Does
`inverseDirection` is a flag per motor in the REF board firmware (`dummy_robot.cpp`). It acts as a **translation layer** between the joint-angle coordinate system and the raw motor position.

### Where It Is Applied
- **Writing positions**: `SetAngle()` and `SetAngleWithVelocityLimit()` — used by `MoveJ()`, `Homing()`, `Resting()`
- **Reading positions**: `UpdateAngleCallback()` — used for angle reporting

### Where It Is NOT Applied
- `set_position_with_time()` (raw motor command, bypasses inversion)
- `set_position()` (raw motor command)
- `set_velocity()` (raw motor command)
- `set_current()` (raw motor command)
- `apply_home_offset()` (just sets encoder to 0)
- `do_calibration()` (encoder calibration)

### Testing Direction
To test the raw motor direction (without `inverseDirection` influence):
```python
dummy0.robot.joint_X.set_position_with_time(-1, 1)  # observe physical direction
dummy0.robot.joint_X.set_position_with_time(1, 1)   # observe physical direction
```

To verify `inverseDirection` is correct: use `Homing()` or `MoveJ()` and check if the arm moves in the expected direction.

---

## 3. `inverseDirection` Settings

### Reference Design (original code, from annotated source)
```cpp
motorJ[1] = new CtrlStepMotor(_hcan, 1, true,  30, -170, 170);  // J1
motorJ[2] = new CtrlStepMotor(_hcan, 2, false, 30, -73, 90);    // J2
motorJ[3] = new CtrlStepMotor(_hcan, 3, true,  30, 35, 180);    // J3
motorJ[4] = new CtrlStepMotor(_hcan, 4, false, 24, -180, 180);  // J4
motorJ[5] = new CtrlStepMotor(_hcan, 5, true,  30, -120, 120);  // J5
motorJ[6] = new CtrlStepMotor(_hcan, 6, true,  50, -720, 720);  // J6
```

### Our Hardware Findings

| Joint | Reference | Our Finding | Status |
|-------|-----------|-------------|--------|
| J1 | true | Untested (doesn't move during Homing) | **NEEDS TESTING** |
| J2 | false | **false is correct** — true causes motor to push backward past REST limit during Homing | **CONFIRMED** |
| J3 | true | **false is correct** — true causes motor to push past 180° limit during Homing | **CONFIRMED** |
| J4 | false | Untested (doesn't move during Homing) | **NEEDS TESTING** |
| J5 | true | Untested (doesn't move during Homing) | **NEEDS TESTING** |
| J6 | true | Untested (doesn't move during Homing) | **NEEDS TESTING** |

### J2 Direction Details (Confirmed)
- `set_position_with_time(-1, 1)` → arm moves **backward** (toward fold/REST)
- `set_position_with_time(1, 1)` → arm moves **forward** (away from fold)
- During `Homing()` with `inverseDirection=true`: motor sent to -6.08 turns (backward) — **WRONG** (should go forward to unfold from REST to L-Pose)
- During `Homing()` with `inverseDirection=false`: motor sent to +6.08 turns (forward) — **CORRECT**

### J3 Direction Details (Confirmed)
- `set_position_with_time(-12, 2)` → upper arm unfolds to almost perpendicular with lower arm
- Raw **negative = unfold** (toward 90°/L-Pose), raw **positive = fold** (toward 180°+)
- During `Homing()` with `inverseDirection=true`: motor sent to +7.5 turns (folding) — **WRONG** (should unfold from REST to L-Pose)
- During `Homing()` with `inverseDirection=false`: motor sent to -7.5 turns (unfolding) — **CORRECT**

### Why Our Hardware Differs From Reference

The direction was wrong **from initial assembly** (not caused by corrupted data or software changes). Three physical factors can reverse motor direction:

#### Factor 1: Motor Coil Wiring (Most Likely)
The 42-motor (NEMA 17 stepper) has two coils: Coil A and Coil B. The TB67H450 dual H-bridge on each motor board drives these with FOC current vectors. If:
- **Coil A and B are swapped** (A pair plugged into B terminals and vice versa), OR
- **One coil's polarity is reversed** (e.g., A+ and A- swapped)

...the motor spins in the opposite direction for the same electrical command. This is a physical wiring difference at assembly time and would explain the wrong direction from first power-on.

#### Factor 2: Encoder Magnet Orientation (N/S Flip)
The MT6816 magnetic encoder reads a diametrically magnetized magnet on the motor shaft. If the magnet is installed with N/S poles flipped (rotated 180° around the diameter axis), encoder readings invert — positive rotation reads as negative. This directly affects the `goDirection` flag during `do_calibration()`.

#### Factor 3: The `goDirection` Flag — The Linchpin
During `do_calibration()`, the firmware determines which direction encoder counts increase when driving the motor with positive current:
```cpp
// encoder_calibrator_base.cpp
goDirection = (endPosition > startPosition) ? 1 : -1;
```
This is baked into the 16,384-entry calibration lookup table. Even if coil wiring is "reversed", `do_calibration()` compensates via `goDirection`, so position tracking works. But the **sign convention** (which direction is "positive turns") will be opposite to the reference design's assumption.

#### Factor 4: Power Button Issue — Unrelated
The power button malfunction (REF board stays on regardless of button position) is unrelated to motor direction. The power circuit is upstream of all logic — a shorted button keeps the board always-on but doesn't affect CAN signals, H-bridge polarity, or encoder readings. Worth fixing for safety (kill switch!) but a separate issue.

#### Summary
Most likely: J2 and J3's motor connectors were plugged in with a different coil ordering or magnet orientation than the reference design assumed. `do_calibration()` correctly compensated via `goDirection`, so raw position tracking worked, but the sign convention was opposite to what the firmware's `inverseDirection` flags expected. Changing `inverseDirection` to `false` aligns software with our actual hardware.

---

## 4. Motor Driver Board Issues

### `erase_configs()` Resets ALL Motor Parameters
After `erase_configs()`, the motor driver board defaults are **much weaker** than required:

| Parameter | Default (after erase) | Required (J1,J4,J5,J6) | Required (J2) | Required (J3) |
|-----------|----------------------|------------------------|---------------|---------------|
| current_limit | 1A | 2A | 2A | 2A |
| dce_kp | 200 | 1000 | 1000 | 1500 |
| dce_kv | 80 | 80 | 80 | 80 |
| dce_ki | 300 | 200 | 200 | 200 |
| dce_kd | 250 | 250 | 200 | 250 |

**Always restore PID and current limits after `erase_configs()`:**
```python
# Current limits for ALL joints
for j in ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']:
    getattr(dummy0.robot, j).set_current_limit(2)

# PID for J1, J4, J5, J6
for j in ['joint_1','joint_4','joint_5','joint_6']:
    getattr(dummy0.robot, j).set_dce_kp(1000)
    getattr(dummy0.robot, j).set_dce_kv(80)
    getattr(dummy0.robot, j).set_dce_ki(200)
    getattr(dummy0.robot, j).set_dce_kd(250)

# PID for J2 (different kd)
dummy0.robot.joint_2.set_dce_kp(1000)
dummy0.robot.joint_2.set_dce_kv(80)
dummy0.robot.joint_2.set_dce_ki(200)
dummy0.robot.joint_2.set_dce_kd(200)

# PID for J3 (higher kp)
dummy0.robot.joint_3.set_dce_kp(1500)
dummy0.robot.joint_3.set_dce_kv(80)
dummy0.robot.joint_3.set_dce_ki(200)
dummy0.robot.joint_3.set_dce_kd(250)
```

### Reboot Loop After `erase_configs()`
- **Symptom:** LED on for ~1 second, brief off, repeat
- **Cause:** Corrupted EEPROM/flash on motor driver board — board keeps rebooting
- **Fix:** Reflash motor driver firmware via SWD (power cycle alone won't fix it)
- **Prevention:** Be cautious with `erase_configs()` — it can brick the motor board if flash write fails

### Motor Driver LED Patterns (from `led_base.cpp`)

| LED 1 Blink Pattern | Motor State | Meaning |
|---------------------|-------------|---------|
| 0 blinks (off) | STATE_STOP / STATE_FINISH | Normal (stopped or position reached) |
| 1 blink per cycle | STATE_NO_CALIB | Encoder calibration data missing |
| 2 blinks per cycle | STATE_STALL | Motor stalled |
| 3 blinks per cycle | STATE_OVERLOAD | Motor exceeding current limit |

LED 0 (motor state): OFF = disabled, solid ON = enabled idle, heartbeat = running.

---

## 5. Reflashing Motor Driver Board via SWD

### SWD Pins
- MCU: STM32F103CBTx
- SWDIO: PA13
- SWCLK: PA14
- GND: any ground pad
- The 6-pin connector (P1, pointing left when CAN connectors face up) contains SWD pins

### Building the 42-Motor Firmware
```bash
cd 2.Firmware/dummy-42motor-fw
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(sysctl -n hw.ncpu)
# Output: build/Ctrl-Step-STM32-fw.bin
```

### Flashing
**Requires:** ST-Link V2 connected to SWD pins, board must be powered on (24V).
```bash
st-flash erase                          # wipe entire flash including corrupted EEPROM
st-flash write build/Ctrl-Step-STM32-fw.bin 0x08000000
```

### After Reflashing a Motor Board
1. Redo `do_calibration()` on that joint (encoder calibration data was erased)
2. Restore PID and current limit parameters (see section 4 above)
3. Redo `apply_home_offset()` or `calibrate_home_offset()`

---

## 6. Data Storage Architecture

| Data | Stored On | Persistence | Erased By |
|------|-----------|-------------|-----------|
| Encoder calibration table | Motor driver flash (separate area) | Survives erase_configs | SWD full erase only |
| Home offset (encoderHomeOffset) | Motor driver EEPROM | Persistent | erase_configs() |
| PID params, current limit | Motor driver EEPROM | Persistent | erase_configs() |
| CAN node ID | Hardware DIP switches | Always | Cannot be erased |
| initPose, currentJoints | REF board RAM (from compiled constants) | Re-initialized on every boot | Reflash REF firmware |
| inverseDirection | REF board firmware (compiled) | Compiled constant | Reflash REF firmware |

**Key insight:** The REF board stores NO persistent config. All persistent data is on the motor driver boards.

---

## 7. Deep Analysis of `calibrate_home_offset()` Function

### Source: `dummy_robot.cpp` lines 199-229

### Full Step-by-Step Trace (with corrected J2=false, J3=false)

**Pre-condition:** User has manually positioned the arm precisely at L-Pose `{0, 0, 90, 0, 0, 0}`.

#### Lines 201-203: Disable control loop, keep motors energized
```cpp
isEnabled = false;
motorJ[ALL]->SetEnable(true);
```
- `isEnabled = false` stops the periodic position update loop (so the arm doesn't fight manual positioning)
- `SetEnable(true)` keeps motors holding position (prevents the arm from falling under gravity)

#### Lines 207-209: Reduce J2/J3 current to 0.5A
```cpp
motorJ[2]->SetCurrentLimit(0.5);
motorJ[3]->SetCurrentLimit(0.5);
```
This serves two purposes:
1. **Safety measure:** If `inverseDirection` is wrong, the motor pushes the wrong way — but at 0.5A it stalls harmlessly instead of slamming the arm into a mechanical stop at full 2A. This is why the first failed attempt "hung" instead of breaking anything.
2. **Physics optimization:** The next move is L-Pose → REST_POSE, which is **gravity-assisted** (the arm naturally wants to fold down). 0.5A is enough because gravity helps.

#### Line 212: First `ApplyPositionAsHome()` — temporary reference
```cpp
motorJ[ALL]->ApplyPositionAsHome();
```
- All motor encoders → 0 at current position (L-Pose)
- This gives the firmware a known starting point to compute motor commands for the next move

#### Lines 216-217: Set coordinate frame to L-Pose
```cpp
initPose = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
currentJoints = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
```
- `initPose = L-Pose` means "motor encoder 0 = L-Pose" (matches what we just set)
- `currentJoints = L-Pose` means "we are currently at L-Pose"

#### Line 218: `Resting()` — the direction verification move
```cpp
Resting();  // → MoveJ(0, -73, 180, 0, 0, 0)
```

This is the critical step. With corrected settings (J2=false, J3=false):

**J2** (L-Pose 0° → REST -73°, arm folds backward):
```
motor_angle = -73 - 0 = -73
inverseDirection=false: -73 (no change)
stepMotorCnt = -73/360 * 30 = -6.08 turns
→ raw -6.08 = backward = arm folds ✓
→ gravity-assisted, 0.5A is sufficient ✓
```

**J3** (L-Pose 90° → REST 180°, elbow folds closed):
```
motor_angle = 180 - 90 = +90
inverseDirection=false: +90 (no change)
stepMotorCnt = +90/360 * 30 = +7.5 turns
→ if raw +7.5 = fold toward 180°, then ✓
→ gravity-assisted, 0.5A is sufficient ✓
```

**J1, J4, J5, J6** all have target = 0 and initPose = 0, so motor_angle = 0. No movement.

The arm smoothly folds from L-Pose into REST_POSE. If directions were wrong, motors would stall safely at 0.5A.

#### Line 222: Second `ApplyPositionAsHome()` — permanent boot reference
```cpp
motorJ[ALL]->ApplyPositionAsHome();
```
- All motor encoders → 0 at REST_POSE
- After reboot: `initPose = REST_POSE` (compiled default) + motors at 0 → self-consistent
- This is the permanent reference that survives reboots

#### Lines 224-225: Restore J2/J3 current to 1A
```cpp
motorJ[2]->SetCurrentLimit(1);
motorJ[3]->SetCurrentLimit(1);
```
Current restored for normal operation.

#### Line 228: Reboot
```cpp
Reboot();
```
Reboots both motor drivers and REF board. After reboot:
- `initPose = REST_POSE = {0, -73, 180, 0, 0, 0}` (from compiled default)
- `currentJoints = REST_POSE` (from compiled default)
- Motor encoders = 0 (from second ApplyPositionAsHome)
- Everything is self-consistent ✓

### Why Two `ApplyPositionAsHome()` Calls

| Call | Position | Purpose |
|------|----------|---------|
| First (line 212) | L-Pose | **Temporary reference** — gives firmware a known origin to compute motor commands for the REST_POSE move |
| Second (line 222) | REST_POSE | **Permanent reference** — sets the boot-up encoder position that matches `initPose = REST_POSE` |

The move between them (L-Pose → REST) serves as a **direction verification test**. If the arm reaches REST correctly, the directions are confirmed correct.

### Why `calibrate_home_offset()` Failed — Complete Root Cause (2026-03-06)

The failure was caused by **two compounding issues**: wrong `inverseDirection` flags AND wrong reduction ratios.

#### Issue 1: Wrong Reduction Ratios
The code had `reduction=30` but actual reducers are 50:1. This caused all movements to be scaled by 30/50 = 60%.

During `calibrate_home_offset()`, the `Resting()` move from L-Pose to REST:
- **J2**: intended -73°, actual movement = 73 × 30/50 = **43.8°** → target = -43.8° (halfway, not REST)
- **J3**: intended +90°, actual movement = 90 × 30/50 = **54°** → target = 144° (halfway, not REST)

#### Issue 2: Gravity + 0.5A Current Limit
With J2/J3 current at 0.5A (safety limit in `calibrate_home_offset()`):
1. The motor targets the **wrong halfway position** (-43.8° for J2)
2. Gravity pulls the arm **past the target** to real REST (-73°)
3. The motor tries to pull the arm **back up** from -73° to -43.8° at only 0.5A
4. 0.5A is too weak to fight gravity in the **upward direction**
5. Motor stuck in **OVERLOAD** (3-blink LED pattern)
6. `IsMoving()` never returns false → `Resting()` **hangs forever**

#### Why It Looked Like It Worked
The arm appeared to fold correctly (reached REST), but this was **gravity doing the work**, not motor control. The motor was actually trying to stop the arm at the halfway point and couldn't.

#### Issue 3: Wrong `inverseDirection` (Earlier Attempt)
The very first attempt also had wrong `inverseDirection` on J2, which sent the motor forward instead of backward. Combined with wrong reduction, this created confusing behavior that led to a cascade of troubleshooting.

#### Lessons Learned
1. **Wrong reduction + gravity + low current limit** creates a deceptive failure mode: the arm moves to the right place (by gravity) but the motors fight to pull it back to the wrong place
2. **Always verify reduction ratios** match physical hardware before calibration
3. **Always verify motor directions** with `set_position_with_time()` before calibration
4. The 0.5A safety current in `calibrate_home_offset()` is designed to prevent damage from wrong directions, but it can't protect against wrong reduction ratios where gravity overshoots the target

**Key takeaway:** After every `do_calibration()`, verify raw motor direction with `set_position_with_time()` before trusting `inverseDirection` settings. The correct `inverseDirection` depends on the motor driver firmware version (see Section 8).

### Physics of the L-Pose → REST Move (Why 0.5A Works)

| Joint | Motion | Gravity Effect | 0.5A Sufficient? |
|-------|--------|----------------|-------------------|
| J2 | 0° → -73° (fold backward) | Gravity **assists** (arm falls back) | Yes |
| J3 | 90° → 180° (fold closed) | Gravity **assists** (upper arm falls) | Yes |

The reverse direction (REST → L-Pose, i.e. `Homing()`) fights gravity and requires the full 2A current limit. This is why `calibrate_home_offset()` moves L-Pose → REST (not the other way) — it's the safe, gravity-assisted direction.

---

## 8. CRITICAL: Motor Driver Firmware Affects `inverseDirection`

The `goDirection` flag determined during `do_calibration()` depends on the **motor driver firmware version**, not just the physical hardware. Our self-built firmware (`dummy-42motor-fw` from this repo) produces the **opposite `goDirection`** compared to the vendor's pre-flashed firmware.

**Evidence:**
- J1, J2, J3 are identical 42-motors with identical coil wiring
- J1, J2 (vendor firmware) → `inverseDirection=true` is correct
- J3 (our firmware, reflashed via SWD) → `inverseDirection=false` is correct
- Only difference: the motor driver firmware

**Rules:**
- Motor driver board with **vendor firmware** → use reference design `inverseDirection` values
- Motor driver board with **our self-built firmware** → likely need to flip `inverseDirection`
- **After any motor driver reflash**: always re-run `do_calibration()`, then test raw direction with `set_position_with_time()` to verify `inverseDirection` is still correct
- The REF board firmware (`inverseDirection` flags) and motor driver firmware version are **coupled** — changing one may require updating the other

See `joint_direction_verification.md` for the full analysis and test procedures.

---

## 9. Reduction Ratio Must Match Physical Hardware

The `reduction` parameter in `CtrlStepMotor` constructor is a compiled constant on the REF board. It must match the actual gearbox ratio of the physical reducer. If wrong, ALL high-level movements (MoveJ, Homing, Resting, calibrate_home_offset) and angle reporting will be scaled incorrectly.

**Our hardware uses 50:1 reducers everywhere** (the reference design used mixed ratios). Corrected values:

```cpp
motorJ[1] = new CtrlStepMotor(_hcan, 1, true,  50, -170, 170);  // was 30
motorJ[2] = new CtrlStepMotor(_hcan, 2, false, 50, -73, 90);    // was 30
motorJ[3] = new CtrlStepMotor(_hcan, 3, false, 50, 35, 180);    // was 30
motorJ[4] = new CtrlStepMotor(_hcan, 4, true,  40, -180, 180);  // was 24 (50×0.8 belt coupling)
motorJ[5] = new CtrlStepMotor(_hcan, 5, false, 50, -120, 120);  // was 30
motorJ[6] = new CtrlStepMotor(_hcan, 6, false, 50, -720, 720);  // already correct
```

J4 uses 40 (not 50) because of a belt/pulley coupling (ratio 0.8) between the 35-motor and the 42-reducer. See `joint_direction_verification.md` Section 8 for details.

Motor driver boards are not affected — they only deal with raw motor turns.

---

## 10. TODO List

- [x] Verify J3 raw direction: negative = unfold, positive = fold — **CONFIRMED**
- [x] Change J2 to `inverseDirection=false` in dummy_robot.cpp — **DONE**
- [x] Change J3 to `inverseDirection=false` in dummy_robot.cpp — **DONE**
- [x] Verify J1, J4, J5, J6 directions — **CONFIRMED** (J4, J5, J6 needed changes)
- [x] Change J4 to `inverseDirection=true` — **DONE**
- [x] Change J5 to `inverseDirection=false` — **DONE**
- [x] Change J6 to `inverseDirection=false` — **DONE**
- [x] Fix reduction ratios (all 50:1, J4=40) — **DONE**
- [x] Rebuild and flash REF firmware — **DONE**
- [x] Test `Homing()` — arm reaches proper L-Pose — **CONFIRMED**
- [x] Test `Resting()` — arm folds back to REST — **CONFIRMED**
- [ ] Fix `calibrate_home_offset()` — 0.5A too low for 50:1 gearbox, needs code change to increase current
- [ ] Investigate power button malfunction on REF board (always on)
