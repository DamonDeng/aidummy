# Guide for Agents Using This Robot Arm

This is a Dummy V2 6-DOF robotic arm controlled via a Python CLI tool.

---

## Connecting

```bash
cd 3.Software/CLI-Tool
python run_shell.py
```

The robot object is available as `dummy0.robot` after connection.

---

## Safe Commands

### Movement Commands

```python
# Move to L-Pose (arm extended: lower arm vertical, upper arm horizontal)
# WARNING: This command takes 15-30 seconds to return. Do not interrupt.
dummy0.robot.homing()

# Move to REST pose (arm fully folded)
# WARNING: This command takes 15-30 seconds to return. Do not interrupt.
dummy0.robot.resting()

# Move joints to specific angles (degrees)
# Arguments: j1, j2, j3, j4, j5, j6
# This only sets the target — it returns quickly.
dummy0.robot.move_j(j1, j2, j3, j4, j5, j6)

# Move end effector to cartesian pose (mm and degrees)
# Arguments: x, y, z, a, b, c
dummy0.robot.move_l(x, y, z, a, b, c)
```

### Speed and Acceleration

```python
# Set joint speed (0-100, default 30 degree/s)
dummy0.robot.set_joint_speed(30)

# Set joint acceleration (0-100)
dummy0.robot.set_joint_acc(50)
```

### Reading Joint Angles

```python
# Read current motor angle for a single joint
dummy0.robot.joint_1.angle
dummy0.robot.joint_2.angle
# ... through joint_6

# Read all joint angles
for j in ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']:
    print(f"{j}: {getattr(dummy0.robot, j).angle}")
```

Note: `joint_X.angle` is the raw motor angle (encoder value converted through reduction ratio). The firmware internally adds `initPose` offset to compute actual joint angles for movement calculations.

After proper home offset calibration, at REST pose all angles read approximately 0.

### Hand (Gripper) Control

```python
# Set hand angle (0-30 degrees)
dummy0.robot.hand.set_angle(15)

# Enable/disable hand
dummy0.robot.hand.set_enable(True)
dummy0.robot.hand.set_enable(False)
```

### Robot Enable/Disable

```python
# Enable motors (required before movement)
dummy0.robot.set_enable(True)

# Disable motors (arm will go limp!)
dummy0.robot.set_enable(False)
```

### RGB LED

```python
dummy0.robot.set_rgb_enable(True)
dummy0.robot.set_rgb_mode(0)
```

---

## Joint Angle Limits

| Joint | Min (degrees) | Max (degrees) |
|-------|--------------|---------------|
| J1 (base rotation) | -170 | 170 |
| J2 (lower arm) | -73 | 90 |
| J3 (upper arm/elbow) | 35 | 180 |
| J4 (wrist roll) | -180 | 180 |
| J5 (wrist pitch) | -120 | 120 |
| J6 (end effector roll) | -720 | 720 |

---

## Key Poses

| Pose | J1 | J2 | J3 | J4 | J5 | J6 | Description |
|------|----|----|----|----|----|----|-------------|
| REST | 0 | -73 | 180 | 0 | 0 | 0 | Fully folded, boot-up position |
| L-Pose | 0 | 0 | 90 | 0 | 0 | 0 | Arm extended (lower vertical, upper horizontal) |

---

## Important Notes

- `homing()` and `resting()` block for 15-30 seconds. They have **no timeout** — if a motor stalls, they hang forever. Power cycle to abort.
- `move_j()` only sets the target and returns quickly. The arm moves asynchronously.
- Always ensure the arm has been properly calibrated (home offset set) before using movement commands. If angles read wildly wrong values at boot, the home offset may need recalibration.
- The robot must be enabled (`set_enable(True)`) before motors will respond to movement commands.

---

## Commands to AVOID (can damage hardware or lose calibration)

Do NOT use these commands unless you know exactly what you're doing:

- `calibrate_home_offset()` — currently broken (0.5A too low for our 50:1 gearboxes), use manual approach instead
- `joint_X.apply_home_offset()` — resets encoder zero point, destroys calibration
- `joint_X.do_calibration()` — runs encoder calibration, motor will spin freely
- `joint_X.erase_configs()` — erases all motor config, can brick the motor driver board
- `joint_X.set_node_id()` — changes CAN address, will lose communication
- `dummy0.robot.reboot()` — reboots all boards, only use when intentional
- `joint_X.set_position_with_time()` — raw motor command, bypasses angle limits and safety checks
- `joint_X.set_position()` — raw motor command, no velocity limit
- `joint_X.set_velocity()` — raw motor command, continuous rotation
- `joint_X.set_current()` — raw current control
