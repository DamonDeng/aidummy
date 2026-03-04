# Dummy X Robot Arm — Python CLI Command Reference

## 1. Starting the Shell

```bash
cd 3.Software/CLI-Tool
python3 run_shell.py
```

The connected board appears as `dummy0` in the interactive shell.

---

## 2. Robot-Level Commands (`dummy0.robot.*`)

### Enable / Disable All Joints

```python
dummy0.robot.set_enable(True)   # enable all 6 joints (motors hold position)
dummy0.robot.set_enable(False)  # disable all 6 joints (motors go limp)
```

### Move to Named Poses

```python
dummy0.robot.homing()    # move all joints to L-pose / home: {J1=0, J2=0, J3=90°, J4=0, J5=0, J6=0}
dummy0.robot.resting()   # move all joints to rest/storage pose: {J1=0, J2=-73°, J3=180°, J4=0, J5=0, J6=0}
```

> **Always call `resting()` before powering off**, so the arm parks at REST_POSE.  
> On next power-on the motor drivers initialize correctly from that stored rest position.

### Move in Joint Space

```python
dummy0.robot.move_j(j1, j2, j3, j4, j5, j6)
# All values in degrees. Example: move to home pose
dummy0.robot.move_j(0, 0, 90, 0, 0, 0)
```

> ✅ `move_j()` enforces per-joint angle limits — safe to use.

### Move in Cartesian Space

```python
dummy0.robot.move_l(x, y, z, a, b, c)
# x/y/z in mm, a/b/c (roll/pitch/yaw) in degrees
dummy0.robot.move_l(200, 0, 300, 0, 0, 0)
```

> ✅ `move_l()` enforces angle limits via inverse kinematics — safe to use.

### Set Joint Speed (for move_j / move_l)

```python
dummy0.robot.set_joint_speed(30)  # speed in degrees/second, default ~30
```

### Calibrate Home Offset (one-time assembly step)

```python
dummy0.robot.calibrate_home_offset()
# ⚠️  Only run when ALL 6 joints are connected AND arm is manually posed in L-pose.
# The board reboots automatically after this — the shell will disconnect.
```

### Board-Level Info

```python
dummy0.serial_number          # board serial number (read-only property)
dummy0.get_temperature()      # REF board MCU temperature in °C
dummy0.get_voltage()          # supply voltage in V
```

---

## 3. Per-Joint Commands (`dummy0.robot.joint_N.*`)

Replace `N` with the joint number 1–6.

### Read Angle

```python
dummy0.robot.joint_1.angle    # current joint angle in degrees (read-only)
```

### Enable / Disable a Single Joint

```python
dummy0.robot.joint_1.set_enable(True)
dummy0.robot.joint_1.set_enable(False)
```

### Move a Single Joint (low-level, no limit check)

```python
dummy0.robot.joint_1.set_position_with_time(pos, vel)
# pos: target position in MOTOR LAPS (not degrees!)
# vel: velocity limit in motor laps/second
```

**Unit conversion formula:**

```
pos = target_degrees / 360 * reduction_ratio

Joint reductions (from firmware):
  J1 (base):        30
  J2 (shoulder):    30
  J3 (elbow):       30
  J4 (wrist roll):  24
  J5 (wrist pitch): 30
  J6 (wrist yaw):   50
```

**Example — move joint 1 to 30°:**

```python
pos = 30 / 360 * 30   # = 2.5 motor laps
dummy0.robot.joint_1.set_position_with_time(2.5, 1.0)
```

> ⚠️ `set_position_with_time()` does **NOT** check angle limits.  
> Prefer `move_j()` for safety. Use this only for low-level testing.

### Encoder Calibration (one-time per motor board)

```python
dummy0.robot.joint_1.do_calibration()
# Motor spins ~1 full revolution forward + backward (~10 seconds).
# Arm can be at any position. Data saved to motor driver flash permanently.
# Only needed once per board, or after erase_configs().
```

### Get Motor Temperature

```python
dummy0.robot.joint_1.get_temperature()   # motor driver MCU temperature in °C
```

### Erase Motor Config (factory reset)

```python
dummy0.robot.joint_1.erase_configs()
# ⚠️  Erases calibration data from flash. You must re-run do_calibration() after this.
```

### Reboot a Single Joint Driver

```python
dummy0.robot.joint_1.reboot()
```

---

## 4. Joint Angle Limits

Enforced by `move_j()` and `move_l()`. **Not** enforced by `set_position_with_time()`.

| Joint | Min (°) | Max (°) | Reduction | Notes |
|-------|---------|---------|-----------|-------|
| J1    | -170    | +170    | 30        | Base rotation |
| J2    | -73     | +90     | 30        | Shoulder — asymmetric (rest pose uses -73°) |
| J3    | +35     | +180    | 30        | Elbow — lower limit > 0° (cannot fully straighten past 35°) |
| J4    | -180    | +180    | 24        | Wrist roll |
| J5    | -120    | +120    | 30        | Wrist pitch |
| J6    | -720    | +720    | 50        | Wrist yaw — 2 full turns each way |

---

## 5. Reference Poses

| Pose | J1 | J2 | J3 | J4 | J5 | J6 | Notes |
|------|----|----|----|----|----|----|-------|
| **Home / L-pose** | 0° | 0° | 90° | 0° | 0° | 0° | Upper arm vertical, forearm horizontal. Use for `calibrate_home_offset()`. |
| **Rest pose** | 0° | -73° | 180° | 0° | 0° | 0° | Compact folded position. Park here before power-off. |

---

## 6. Typical Session Workflow

```python
# --- Every power-on ---
dummy0.robot.set_enable(True)
dummy0.robot.homing()              # move from REST_POSE → home/L-pose

# --- Work ---
dummy0.robot.move_j(0, 0, 90, 0, 0, 0)   # example command
dummy0.robot.set_joint_speed(20)           # slow down if needed

# --- Before power-off ---
dummy0.robot.resting()             # fold to REST_POSE
# Now safe to power off
```

---

## 7. One-Time Setup (new boards or after erase_configs)

```python
# Step 1: Encoder calibration — arm can be at any position
dummy0.robot.joint_1.do_calibration()   # repeat for joint_2 … joint_6
# Wait ~10 seconds per joint

# Step 2: Home offset calibration — manually pose arm to L-pose first
# (J1=0, J2=0, J3=90°, J4-J6=0)
# Then, while physically supporting the arm in L-pose:
dummy0.robot.calibrate_home_offset()
# Board reboots automatically — reconnect shell after ~3 seconds

# Step 3: Verify
dummy0.robot.joint_1.angle   # should read ~0
dummy0.robot.joint_2.angle   # should read ~0
dummy0.robot.joint_3.angle   # should read ~0  (motor laps, ~0 from home)
```

---

## 8. HTTP Server API (`3.Software/robot-server/robot_server.py`)

A FastAPI server that holds the USB connection persistently and exposes a REST API
callable from Node.js, Python `requests`, `curl`, or any HTTP client.

### Starting the Server

```bash
cd /path/to/dummy_v2_ren
python3 3.Software/robot-server/robot_server.py
# → Listening on http://127.0.0.1:3001
```

Interactive API docs (auto-generated by FastAPI):
**http://127.0.0.1:3001/docs**

---

### REST Endpoints

#### Robot-level

| Method | Path | Body (JSON) | Description |
|--------|------|-------------|-------------|
| POST | `/robot/enable` | `{"enable": true}` | Enable / disable all joints |
| POST | `/robot/homing` | — | Move to L-pose / home |
| POST | `/robot/resting` | — | Move to rest/storage pose |
| POST | `/robot/move_j` | `{"j1":0,"j2":0,"j3":90,"j4":0,"j5":0,"j6":0}` | Move in joint space (checks limits) |
| POST | `/robot/move_l` | `{"x":…,"y":…,"z":…,"a":…,"b":…,"c":…}` | Move in Cartesian space |
| POST | `/robot/set_joint_speed` | `{"speed": 50}` | Set default joint speed (%) |
| POST | `/robot/calibrate_home_offset` | — | Run home offset calibration |
| GET | `/robot/angles` | — | Read all 6 joint angles (motor laps) |
| GET | `/robot/info` | — | Serial number, temperature, voltage |

#### Per-joint (`n` = 1–6)

| Method | Path | Body (JSON) | Description |
|--------|------|-------------|-------------|
| GET | `/robot/joint/{n}/angle` | — | Read single joint angle (motor laps) |
| GET | `/robot/joint/{n}/temperature` | — | Read motor temperature (°C) |
| POST | `/robot/joint/{n}/enable` | `{"enable": true}` | Enable / disable single joint |
| POST | `/robot/joint/{n}/set_position_with_time` | `{"pos": 2.5, "vel": 2.0}` | Raw move — **no limit check** |
| POST | `/robot/joint/{n}/do_calibration` | — | Encoder calibration (~10 s) |
| POST | `/robot/joint/{n}/reboot` | — | Reboot motor driver |
| POST | `/robot/joint/{n}/erase_configs` | — | Erase motor flash config |

> **`pos` unit:** motor laps = `target_degrees / 360 * reduction`
> **`vel` unit:** motor laps per second

---

### Node.js Examples

```js
const BASE = 'http://localhost:3001';

// Enable all joints
await fetch(`${BASE}/robot/enable`, {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({ enable: true })
});

// Move to home (L-pose)
await fetch(`${BASE}/robot/homing`, { method: 'POST' });

// Move joints (joint-space, limits enforced)
await fetch(`${BASE}/robot/move_j`, {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({ j1: 0, j2: 0, j3: 90, j4: 0, j5: 0, j6: 0 })
});

// Read all angles
const res = await fetch(`${BASE}/robot/angles`);
const { j1, j2, j3, j4, j5, j6 } = await res.json();

// WebSocket: live angle stream at ~10 Hz
const ws = new WebSocket('ws://localhost:3001/robot/stream');
ws.on('message', (data) => {
  const angles = JSON.parse(data);
  console.log(angles); // { j1: 0.01, j2: -0.02, j3: 0.00, j4: 0.00, j5: 0.00, j6: 0.00 }
});
```

---

### curl Examples

```bash
# Enable all joints
curl -X POST http://localhost:3001/robot/enable \
  -H "Content-Type: application/json" -d '{"enable": true}'

# Move to home
curl -X POST http://localhost:3001/robot/homing

# Read all angles
curl http://localhost:3001/robot/angles

# Move joint 3 raw (low-level, no limit check)
# pos = 90° / 360 * 30 = 7.5 motor laps
curl -X POST http://localhost:3001/robot/joint/3/set_position_with_time \
  -H "Content-Type: application/json" -d '{"pos": 7.5, "vel": 2.0}'
```

