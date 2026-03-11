#!/usr/bin/env python3
"""
Dummy X Robot Arm — FastAPI HTTP/WebSocket server

Bridges the Fibre USB protocol to a REST API callable from Node.js (or any HTTP client).

Usage:
    pip3 install fastapi uvicorn
    python3 robot_server.py          # real hardware via USB
    python3 robot_server.py --sim    # simulation mode (no hardware needed)
    SIM=1 python3 robot_server.py    # same, via env var

Node.js example:
    const res = await fetch('http://localhost:3001/robot/move_j', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ j1: 0, j2: 0, j3: 90, j4: 0, j5: 0, j6: 0 })
    });
    const data = await res.json();  // { "ok": true }

Interactive API docs (auto-generated):
    http://127.0.0.1:3001/docs
"""

import sys
import os
import asyncio
import time
import threading
import json
import re
import math
from datetime import datetime, timezone
from pathlib import Path
from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import Response
from pydantic import BaseModel
from typing import Optional
from ik_solver import solve_ik_batch, solve_ik

# ── Simulation mode flag ──────────────────────────────────────────────────────
SIM_MODE = "--sim" in sys.argv or os.environ.get("SIM", "").lower() in ("1", "true", "yes")

# ── Real hardware import (skipped in sim mode) ────────────────────────────────
if not SIM_MODE:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'CLI-Tool'))
    import fibre


# ══════════════════════════════════════════════════════════════════════════════
# Mock / Simulation layer
# Mirrors the exact object interface that the real fibre USB device exposes,
# so every endpoint above works identically in sim mode.
# ══════════════════════════════════════════════════════════════════════════════

# Joint config mirrors dummy_robot.cpp constructor
_JOINT_CFG = [
    # (reduction, angle_min, angle_max)
    (30, -170.0,  170.0),   # J1 – base rotation
    (30,  -73.0,   90.0),   # J2 – shoulder
    (30,   35.0,  180.0),   # J3 – elbow
    (24, -180.0,  180.0),   # J4 – wrist roll
    (30, -120.0,  120.0),   # J5 – wrist pitch
    (50, -720.0,  720.0),   # J6 – wrist yaw
]

_HOME_POSE   = [0.0,   0.0,  90.0, 0.0, 0.0, 0.0]
_REST_POSE   = [0.0, -73.0, 180.0, 0.0, 0.0, 0.0]
_Z_HOME_POSE = [0.0, -45.0, 140.0, 0.0, 0.0, 0.0]  # lower arm -45°, upper arm ~horizontal
_DEFAULT_SPEED_DEG_S = 30.0   # degrees per second


class MockJoint:
    """
    Simulates one CtrlStepMotor joint.
    Angles move smoothly toward the target at `speed` deg/s.
    The `tick(dt)` method is called by the background simulation loop.
    """

    def __init__(self, index: int):
        reduction, angle_min, angle_max = _JOINT_CFG[index]
        self.index = index
        self.reduction = reduction
        self.angle_min = angle_min
        self.angle_max = angle_max

        self.angle: float = _HOME_POSE[index]      # current angle (deg)
        self._target: float = _HOME_POSE[index]    # goal angle (deg)
        self._speed: float = _DEFAULT_SPEED_DEG_S  # deg/s
        self.enabled: bool = False
        self.temperature: float = 35.0 + index * 1.5   # realistic-ish °C per joint

    # ── Public API (mirrors CtrlStepMotor protocol definitions) ──────────────

    def set_enable(self, enable: bool):
        self.enabled = enable

    def set_position_with_time(self, pos: float, vel: float):
        """Low-level move: pos in motor laps, vel in motor laps/s. No limit check."""
        target_deg = pos / self.reduction * 360.0
        self._target = target_deg

    def do_calibration(self):
        """Simulate ~10 s calibration (instantaneous in sim)."""
        # In real hardware this spins the motor. Here we just note it happened.
        pass

    def get_temperature(self) -> float:
        return self.temperature

    def reboot(self):
        self.enabled = False

    def erase_configs(self):
        pass

    def set_dce_kp(self, val: int): pass
    def set_dce_kv(self, val: int): pass
    def set_dce_ki(self, val: int): pass
    def set_dce_kd(self, val: int): pass

    # ── Simulation internals ──────────────────────────────────────────────────

    def set_target(self, angle_deg: float, speed_deg_s: float):
        """Called by MockRobotInner to command a safe, limit-checked move."""
        self._target = max(self.angle_min, min(self.angle_max, angle_deg))
        self._speed = max(1.0, speed_deg_s)

    def is_moving(self) -> bool:
        return abs(self._target - self.angle) > 0.05  # 0.05° dead-band

    def tick(self, dt: float):
        """Advance angle toward target. Called at ~50 Hz by the sim loop."""
        delta = self._target - self.angle
        max_step = self._speed * dt
        if abs(delta) <= max_step:
            self.angle = self._target
        else:
            self.angle += max_step * (1.0 if delta > 0 else -1.0)


class MockRobotInner:
    """
    Simulates the `dummy.robot` sub-object.
    Mirrors the Fibre protocol properties exposed by dummy_robot.h.
    """

    def __init__(self):
        self.joint_1 = MockJoint(0)
        self.joint_2 = MockJoint(1)
        self.joint_3 = MockJoint(2)
        self.joint_4 = MockJoint(3)
        self.joint_5 = MockJoint(4)
        self.joint_6 = MockJoint(5)
        self._joints = [
            self.joint_1, self.joint_2, self.joint_3,
            self.joint_4, self.joint_5, self.joint_6,
        ]
        self._speed: float = _DEFAULT_SPEED_DEG_S
        self.enabled: bool = False

    # ── Public API ────────────────────────────────────────────────────────────

    def set_enable(self, enable: bool):
        self.enabled = enable
        for j in self._joints:
            j.set_enable(enable)

    def homing(self):
        """Move all joints to L-pose / home."""
        for i, j in enumerate(self._joints):
            j.set_target(_HOME_POSE[i], self._speed)

    def resting(self):
        """Move all joints to rest/storage pose."""
        for i, j in enumerate(self._joints):
            j.set_target(_REST_POSE[i], self._speed)

    def move_j(self, j1: float, j2: float, j3: float,
               j4: float, j5: float, j6: float):
        """Joint-space move with limit check (mirrors MoveJ in firmware)."""
        targets = [j1, j2, j3, j4, j5, j6]
        valid = all(
            self._joints[i].angle_min <= targets[i] <= self._joints[i].angle_max
            for i in range(6)
        )
        if not valid:
            return  # silently drop, same as firmware
        for i, j in enumerate(self._joints):
            j.set_target(targets[i], self._speed)

    def move_l(self, x: float, y: float, z: float,
               a: float, b: float, c: float):
        """
        Cartesian move — IK not implemented in sim.
        Logs the target; no motion occurs.
        """
        print(f"[SIM] move_l({x:.1f}, {y:.1f}, {z:.1f}, {a:.1f}, {b:.1f}, {c:.1f})"
              " — IK not implemented in sim mode, command ignored.")

    def set_joint_speed(self, speed: float):
        """Set speed (deg/s). Mirrors set_joint_speed in firmware."""
        self._speed = max(1.0, min(100.0, speed))

    def calibrate_home_offset(self):
        """Simulate home-offset calibration: zero all angles."""
        for j in self._joints:
            j.angle = 0.0
            j._target = 0.0

    # ── Simulation internals ──────────────────────────────────────────────────

    def tick(self, dt: float):
        for j in self._joints:
            j.tick(dt)

    def is_moving(self) -> bool:
        return any(j.is_moving() for j in self._joints)


class MockDummy:
    """
    Top-level simulation object — mirrors the fibre `dummy` device handle.
    Exposes `.robot`, `.serial_number`, `.get_temperature()`, `.get_voltage()`.
    """

    def __init__(self):
        self.robot = MockRobotInner()
        self.serial_number = "SIM-001"
        self._voltage: float = 24.1
        self._temperature: float = 42.0

    def get_temperature(self) -> float:
        return self._temperature

    def get_voltage(self) -> float:
        return self._voltage


# ── Global robot connection ───────────────────────────────────────────────────
dummy = None
_sim_task = None      # background simulation loop handle
_stop_requested: bool = False   # set by /robot/stop to interrupt play_sequence

# ── Server mode ───────────────────────────────────────────────────────────────
# high_level (default): only safe motion commands are accessible.
# low_level: additionally exposes tuning and diagnostic commands (DCE params,
#            raw calibration, etc.). Switch deliberately; revert when done.
_server_mode: str = "high_level"

def _check_low_level():
    """Call at the top of any low-level-only endpoint to enforce mode gating."""
    if _server_mode != "low_level":
        raise HTTPException(
            status_code=403,
            detail="This endpoint is only available in low_level mode. "
                   "POST /server/mode {\"mode\": \"low_level\"} to enable."
        )

# ── DCE parameter mirror ──────────────────────────────────────────────────────
# Firmware exposes set_dce_* but no get_dce_* — we mirror values server-side.
# Initialised to firmware defaults (kp=200, kv=80, ki=300, kd=250).
_DCE_DEFAULTS = {"kp": 200, "kv": 80, "ki": 300, "kd": 250}
_dce_params: dict[int, dict] = {n: dict(_DCE_DEFAULTS) for n in range(1, 7)}


class _StopRequested(Exception):
    """Internal signal: raised inside execute_steps when /robot/stop is called."""
    pass


# ── Sequence database ─────────────────────────────────────────────────────────
_SEQ_FILE  = Path(__file__).parent / "sequences.json"
_seq_lock  = threading.Lock()
_sequences: dict = {}   # in-memory cache; flushed to disk on every write

_SEQ_NAME_RE = re.compile(r'^[a-z0-9][a-z0-9_-]{0,63}$')   # lowercase, hyphens, underscores

# ── TCP (Tool Center Point) offset ───────────────────────────────────────────
# When a tool is attached to J6, its tip is `_tool_length_mm` beyond the wrist.
# move_l automatically back-calculates the required wrist position so the TOOL
# TIP lands at the user-specified (x, y, z).  Set to 0.0 when no tool attached.
_tool_length_mm: float = 0.0

def _euler_to_rot(a_deg: float, b_deg: float, c_deg: float):
    """
    Build a 3×3 rotation matrix from ZYX Euler angles (degrees).
    Convention matches firmware: a=roll(X), b=pitch(Y), c=yaw(Z).
    Returns a flat list[9] in row-major order (same as firmware R06 layout).
    """
    a = math.radians(a_deg)
    b = math.radians(b_deg)
    c = math.radians(c_deg)
    ca, sa = math.cos(a), math.sin(a)
    cb, sb = math.cos(b), math.sin(b)
    cc, sc = math.cos(c), math.sin(c)
    # R = Rz(c) @ Ry(b) @ Rx(a)
    return [
        cb*cc,          cc*sa*sb - ca*sc,   ca*cc*sb + sa*sc,
        cb*sc,          ca*cc + sa*sb*sc,   ca*sb*sc - cc*sa,
        -sb,            cb*sa,              ca*cb,
    ]

def _apply_tcp(x: float, y: float, z: float,
               a: float, b: float, c: float):
    """
    Adjust the requested TOOL TIP position (x, y, z) to the required WRIST
    position that the firmware IK should target.

    The tool extends along the local Z-axis of J6.  In world coordinates that
    direction is the third column of the end-effector rotation matrix R:
        offset_world = R × [0, 0, tool_length]
    So wrist_target = tool_tip_target - offset_world.
    """
    if _tool_length_mm == 0.0:
        return x, y, z
    R = _euler_to_rot(a, b, c)
    # Third column of R (local Z axis in world frame) = R[2], R[5], R[8]
    wx = x - R[2] * _tool_length_mm
    wy = y - R[5] * _tool_length_mm
    wz = z - R[8] * _tool_length_mm
    return wx, wy, wz


def _load_sequences():
    """Load sequences from disk into _sequences cache. Called at startup."""
    global _sequences
    if _SEQ_FILE.exists():
        with open(_SEQ_FILE, "r") as f:
            _sequences = json.load(f)
        print(f"✓  Loaded {len(_sequences)} sequence(s) from {_SEQ_FILE.name}")
    else:
        _sequences = {}
        print("✓  No sequences.json found — starting with empty library.")


def _save_sequences():
    """Flush _sequences cache to disk. Must be called inside _seq_lock."""
    with open(_SEQ_FILE, "w") as f:
        json.dump(_sequences, f, indent=2)


def _now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


async def _sim_loop():
    """Background task: ticks the mock robot at ~50 Hz."""
    last = time.monotonic()
    while True:
        await asyncio.sleep(0.02)   # 50 Hz
        now = time.monotonic()
        dt = now - last
        last = now
        if dummy is not None:
            dummy.robot.tick(dt)


@asynccontextmanager
async def lifespan(app: FastAPI):
    global dummy, _sim_task

    if SIM_MODE:
        print("🤖  Simulation mode — no hardware required.")
        dummy = MockDummy()
        _sim_task = asyncio.create_task(_sim_loop())
        print("✓  Mock robot ready.")
    else:
        print("Connecting to robot via USB (timeout 90 s)...")
        loop = asyncio.get_event_loop()
        dummy = await loop.run_in_executor(
            None, lambda: fibre.find_any("usb", timeout=90)
        )
        if dummy is None:
            print("⚠  Robot not found. Endpoints will return 503 until connected.")
        else:
            print("✓  Robot connected.")

    yield

    if _sim_task is not None:
        _sim_task.cancel()
    dummy = None


# Load sequence library on import (before first request)
_load_sequences()


app = FastAPI(
    title="Dummy Robot Arm API",
    version="1.0.0",
    lifespan=lifespan,
    description="Set `SIM=1` or pass `--sim` to start without hardware.",
)
app.add_middleware(
    CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"]
)


def get_robot():
    if dummy is None:
        raise HTTPException(status_code=503, detail="Robot not connected")
    return dummy


# ── Request models ────────────────────────────────────────────────────────────
class EnableBody(BaseModel):
    enable: bool


class MoveJBody(BaseModel):
    j1: float
    j2: float
    j3: float
    j4: float
    j5: float
    j6: float


class MoveLBody(BaseModel):
    x: float
    y: float
    z: float
    a: float
    b: float
    c: float


class SetToolBody(BaseModel):
    tool_length_mm: float   # distance from J6 flange to tool tip, in mm. 0 = no tool.


class SetModeBody(BaseModel):
    mode: str  # "high_level" or "low_level"


class DceParamsBody(BaseModel):
    kp: Optional[int] = None  # position error → direct output (stiffness)
    kv: Optional[int] = None  # velocity error → integral (dynamic damping memory)
    ki: Optional[int] = None  # position error → integral (holds against gravity)
    kd: Optional[int] = None  # velocity error → direct output (damping)


class MoveLPathBody(BaseModel):
    poses: list[list[float]]  # list of [x,y,z,a,b,c] waypoints (mm + degrees)
    step_delay_ms: float = 80.0  # milliseconds between MoveJ commands (default 80ms)
    tcp_apply: bool = True        # apply active tool TCP offset to each waypoint


class SpeedBody(BaseModel):
    speed: float


class SoftMoveBody(BaseModel):
    speed:            float = 15.0   # deg/s — default slower for safety
    settle_threshold: float = 0.5    # degrees — "close enough" to target
    settle_timeout:   float = 20.0   # max seconds to wait for settling


class CreateSequenceBody(BaseModel):
    name:             str
    description:      str   = ""
    speed:            float = 20.0
    settle_threshold: float = 0.5
    settle_timeout:   float = 15.0
    steps:            list  # list of SequenceStep-compatible dicts


class UpdateSequenceBody(BaseModel):
    description:      str   | None = None
    speed:            float | None = None
    settle_threshold: float | None = None
    settle_timeout:   float | None = None
    steps:            list  | None = None   # None = keep existing steps


class PlaySequenceOverrideBody(BaseModel):
    speed:            float | None = None   # override saved speed
    settle_threshold: float | None = None
    settle_timeout:   float | None = None


class JointPosBody(BaseModel):
    pos: float
    vel: float


# ── Helper ────────────────────────────────────────────────────────────────────
def get_joint(n: int):
    if n < 1 or n > 6:
        raise HTTPException(status_code=400, detail="Joint number must be 1-6")
    return getattr(get_robot().robot, f"joint_{n}")


# ── Robot-level endpoints ─────────────────────────────────────────────────────
@app.post("/robot/enable")
def set_enable(body: EnableBody):
    get_robot().robot.set_enable(body.enable)
    return {"ok": True}


@app.post("/robot/homing")
def soft_home(body: SoftMoveBody = SoftMoveBody()):
    """
    Soft home: moves the arm to the home pose [0, 0, 90, 0, 0, 0] using
    move_j + settle detection. Replaces the buggy firmware homing() which
    cannot reliably detect completion.

    - Clears any previous stop flag (explicit command = intentional move).
    - Motors are enabled automatically.
    - Default speed: 15 °/s (safe, gentle).
    - Waits until all joints settle within 0.5° of target (or timeout).
    - Respects /robot/stop: exits settle loop early and returns stopped=true.
    """
    global _stop_requested
    _stop_requested = False   # explicit command clears any prior stop

    d = get_robot()
    r = d.robot
    r.set_enable(True)
    r.set_joint_speed(body.speed)

    target_phys = list(_HOME_POSE)   # [0, 0, 90, 0, 0, 0]
    r.move_j(*target_phys)
    target_enc = _physical_to_encoder(target_phys)
    settle_s = _wait_for_settle(r, target_enc, body.settle_threshold, body.settle_timeout,
                                check_stop=True)

    if _stop_requested:
        phys_now = _encoder_to_physical(_read_angles(r))
        return {
            "ok":        False,
            "stopped":   True,
            "frozen_at": [round(a, 2) for a in phys_now],
            "settle_s":  round(settle_s, 2),
        }

    return {
        "ok":       True,
        "pose":     "home",
        "target":   target_phys,
        "angles":   [round(a, 2) for a in _encoder_to_physical(_read_angles(r))],
        "settle_s": round(settle_s, 2),
    }


@app.post("/robot/z_home")
def soft_z_home(body: SoftMoveBody = SoftMoveBody()):
    """
    Z-home: a natural resting pose that looks like a Z from the side.
    Lower arm tilted back at -45° (J2), upper arm approximately horizontal (J3=140°).
    Formula: upper arm angle from horizontal = J2 + J3 - 90°  → -45 + 140 - 90 = 5° (slightly above horizontal)

    - Clears any previous stop flag (explicit command = intentional move).
    - Motors are enabled automatically.
    - Default speed: 15 °/s (safe, gentle).
    - Waits until all joints settle within 0.5° of target (or timeout).
    - Respects /robot/stop: exits settle loop early and returns stopped=true.
    """
    global _stop_requested
    _stop_requested = False

    d = get_robot()
    r = d.robot
    r.set_enable(True)
    r.set_joint_speed(body.speed)

    target_phys = list(_Z_HOME_POSE)   # [0, 0, 60, 0, 0, 0]
    r.move_j(*target_phys)
    target_enc = _physical_to_encoder(target_phys)
    settle_s = _wait_for_settle(r, target_enc, body.settle_threshold, body.settle_timeout,
                                check_stop=True)

    if _stop_requested:
        phys_now = _encoder_to_physical(_read_angles(r))
        return {
            "ok":        False,
            "stopped":   True,
            "frozen_at": [round(a, 2) for a in phys_now],
            "settle_s":  round(settle_s, 2),
        }

    return {
        "ok":       True,
        "pose":     "z_home",
        "target":   target_phys,
        "angles":   [round(a, 2) for a in _encoder_to_physical(_read_angles(r))],
        "settle_s": round(settle_s, 2),
    }


@app.post("/robot/resting")
def soft_rest(body: SoftMoveBody = SoftMoveBody()):
    """
    Soft rest: moves the arm to the rest/storage pose [0, -73, 180, 0, 0, 0]
    using move_j + settle detection. Replaces the buggy firmware resting()
    which cannot reliably detect completion.

    - Clears any previous stop flag (explicit command = intentional move).
    - Motors are enabled automatically.
    - Default speed: 15 °/s (safe, gentle).
    - Waits until all joints settle within 0.5° of target (or timeout).
    - Respects /robot/stop: exits settle loop early and returns stopped=true.
    """
    global _stop_requested
    _stop_requested = False   # explicit command clears any prior stop

    d = get_robot()
    r = d.robot
    r.set_enable(True)
    r.set_joint_speed(body.speed)

    target_phys = list(_REST_POSE)   # [0, -73, 180, 0, 0, 0]
    r.move_j(*target_phys)
    target_enc = _physical_to_encoder(target_phys)
    settle_s = _wait_for_settle(r, target_enc, body.settle_threshold, body.settle_timeout,
                                check_stop=True)

    if _stop_requested:
        phys_now = _encoder_to_physical(_read_angles(r))
        return {
            "ok":        False,
            "stopped":   True,
            "frozen_at": [round(a, 2) for a in phys_now],
            "settle_s":  round(settle_s, 2),
        }

    return {
        "ok":       True,
        "pose":     "rest",
        "target":   target_phys,
        "angles":   [round(a, 2) for a in _encoder_to_physical(_read_angles(r))],
        "settle_s": round(settle_s, 2),
    }


@app.post("/robot/stop")
def stop():
    """
    Freeze the arm in its current position.

    - Sets a flag that interrupts any active play_sequence between steps.
    - Re-issues the current joint angles as the new move target so the arm
      holds its pose — motors stay ENABLED, the arm does NOT fall.
    - Returns the frozen physical angles for logging.
    """
    global _stop_requested
    _stop_requested = True

    d = get_robot()
    r = d.robot

    # Read current encoder angles → convert to physical → re-send as target
    enc_angles   = _read_angles(r)
    phys_angles  = _encoder_to_physical(enc_angles)
    r.move_j(*phys_angles)

    return {"ok": True, "frozen_at": [round(a, 2) for a in phys_angles]}


@app.post("/robot/move_j")
def move_j(body: MoveJBody):
    get_robot().robot.move_j(body.j1, body.j2, body.j3, body.j4, body.j5, body.j6)
    return {"ok": True}


@app.post("/robot/move_l")
def move_l(body: MoveLBody):
    """
    Cartesian move to a tip position.
    x, y, z are in MILLIMETRES (the firmware's IK solver handles mm internally).
    a, b, c are wrist orientation in DEGREES.

    If a tool is configured (POST /robot/tool), x/y/z refer to the TOOL TIP.
    The server automatically back-calculates the required wrist position.

    Returns ok=false with detail if the firmware's IK finds no valid solution.
    """
    r = get_robot().robot
    # Apply TCP offset: convert tool-tip target → wrist target
    wx, wy, wz = _apply_tcp(body.x, body.y, body.z, body.a, body.b, body.c)
    # Firmware IK expects mm — pass directly, no conversion needed
    result = r.move_l(wx, wy, wz, body.a, body.b, body.c)
    if result is False:
        return {"ok": False, "detail": "IK failed — no valid joint solution for this pose"}
    return {"ok": True}


@app.post("/robot/set_joint_speed")
def set_joint_speed(body: SpeedBody):
    get_robot().robot.set_joint_speed(body.speed)
    return {"ok": True}


# ── Tool (TCP) configuration ──────────────────────────────────────────────────

@app.get("/robot/tool")
def get_tool():
    """Return the current tool length setting."""
    return {
        "tool_length_mm": _tool_length_mm,
        "active": _tool_length_mm != 0.0,
    }


@app.post("/robot/tool")
def set_tool(body: SetToolBody):
    """
    Set the tool length (distance from J6 flange to tool tip) in millimetres.
    Use 0.0 to disable TCP compensation (no tool attached).

    After setting this, all move_l calls will automatically target the TOOL TIP,
    not the wrist flange.
    """
    global _tool_length_mm
    if body.tool_length_mm < 0:
        raise HTTPException(status_code=400, detail="tool_length_mm must be >= 0")
    _tool_length_mm = body.tool_length_mm
    return {
        "ok": True,
        "tool_length_mm": _tool_length_mm,
        "active": _tool_length_mm != 0.0,
    }


@app.post("/robot/move_l_path")
def move_l_path(body: MoveLPathBody):
    """
    Stream a Cartesian path as a rapid sequence of MoveJ commands.

    Unlike move_l (which is IK + MoveJ, fire-and-forget), this endpoint:
    1. Pre-computes IK for all waypoints in Python using the analytical solver.
    2. Chains IK solutions so each step uses the previous solution as the
       reference — ensures consistent elbow/shoulder configuration throughout.
    3. Streams the resulting MoveJ commands to the arm with step_delay_ms
       between each, allowing the PID controller to smoothly "chase" the path.

    Waypoints: list of [x, y, z, a, b, c]  (mm + degrees).
    TCP offset is applied to each waypoint if a tool is configured.

    Returns: ok, steps_sent, steps_failed (IK failures skipped with a warning).
    """
    if not body.poses:
        raise HTTPException(status_code=400, detail="poses list is empty")
    for i, p in enumerate(body.poses):
        if len(p) != 6:
            raise HTTPException(status_code=400,
                detail=f"pose[{i}] must have exactly 6 values, got {len(p)}")

    r = get_robot().robot

    # Read current joint angles to seed the IK chain
    raw = _read_angles(r)
    current_j = _encoder_to_physical(raw)

    # Apply TCP offset to each waypoint if needed
    poses_wrist = []
    for p in body.poses:
        if body.tcp_apply and _tool_length_mm > 0:
            wx, wy, wz = _apply_tcp(p[0], p[1], p[2], p[3], p[4], p[5])
            poses_wrist.append((wx, wy, wz, p[3], p[4], p[5]))
        else:
            poses_wrist.append(tuple(p))

    # Batch IK — 3-4ms for 90 poses
    solutions = solve_ik_batch(poses_wrist, start_joints=current_j)

    # Stream MoveJ commands
    sent, failed = 0, 0
    delay_s = body.step_delay_ms / 1000.0
    for sol in solutions:
        if sol is None:
            failed += 1
            continue
        r.move_j(*sol)
        sent += 1
        if delay_s > 0:
            time.sleep(delay_s)

    return {"ok": True, "steps_sent": sent, "steps_failed": failed}


@app.post("/robot/calibrate_home_offset")
def calibrate_home_offset():
    get_robot().robot.calibrate_home_offset()
    return {"ok": True}


# ── Server mode ───────────────────────────────────────────────────────────────

@app.get("/server/mode")
def get_server_mode():
    """
    Return the current server mode.

    Modes:
      high_level (default) — safe motion commands only
      low_level            — additionally exposes tuning and diagnostic endpoints
    """
    return {"mode": _server_mode}


@app.post("/server/mode")
def set_server_mode(body: SetModeBody):
    """
    Switch server mode.

      POST /server/mode {"mode": "low_level"}   → enables tuning endpoints
      POST /server/mode {"mode": "high_level"}  → re-locks them

    ⚠️  Low-level mode exposes commands that can modify controller parameters.
        Switch back to high_level when tuning is complete.
    """
    global _server_mode
    if body.mode not in ("high_level", "low_level"):
        raise HTTPException(status_code=400,
            detail="mode must be 'high_level' or 'low_level'")
    _server_mode = body.mode
    return {"ok": True, "mode": _server_mode}


# ── Per-joint DCE tuning (low-level only) ────────────────────────────────────

@app.get("/robot/joint/{n}/dce")
def get_dce(n: int):
    """
    Return the current (server-mirrored) DCE parameters for joint n (1–6).

    Note: the firmware has no get_dce_* commands, so these values reflect
    what was last set via POST /robot/joint/{n}/dce. On server restart they
    reset to firmware defaults: kp=200, kv=80, ki=300, kd=250.

    What each parameter does (DCE = Dynamic Control Error controller):
      kp — position error → direct current output (stiffness / responsiveness)
      kv — velocity error → integral accumulator (dynamic damping memory)
      ki — position error → integral accumulator (gravity hold / steady-state)
      kd — velocity error → direct current output (instantaneous damping)

    Formula (runs at 20 kHz on STM32):
      output_mA = (kp × pos_err + integral + kd × vel_err) / 1024
      integral  += ki × pos_err + kv × vel_err   (per tick, clamped)
    """
    if n < 1 or n > 6:
        raise HTTPException(status_code=400, detail="joint n must be 1–6")
    return {"joint": n, **_dce_params[n]}


@app.post("/robot/joint/{n}/dce")
def set_dce(n: int, body: DceParamsBody):
    """
    Update DCE parameters for joint n (1–6). Low-level mode required.

    Only the fields you supply are updated; omitted fields keep their current value.

    Tuning guidelines:
      Chattering / oscillation during motion:
        → lower ki (less integral wind-up) and/or raise kd (more damping)
      Sluggish / doesn't hold position under load:
        → raise ki (more gravity hold) and/or raise kp
      Overshoot on large moves:
        → lower kp and/or raise kd

    Safety: changes take effect immediately on the live joint. Start with
    small adjustments (±20–50 per parameter) and test at low speed.
    Reboot the joint board to restore firmware defaults.
    """
    _check_low_level()
    if n < 1 or n > 6:
        raise HTTPException(status_code=400, detail="joint n must be 1–6")

    joint = get_joint(n)
    updated = {}

    if body.kp is not None:
        if body.kp < 0:
            raise HTTPException(status_code=400, detail="kp must be >= 0")
        joint.set_dce_kp(body.kp)
        _dce_params[n]["kp"] = body.kp
        updated["kp"] = body.kp

    if body.kv is not None:
        if body.kv < 0:
            raise HTTPException(status_code=400, detail="kv must be >= 0")
        joint.set_dce_kv(body.kv)
        _dce_params[n]["kv"] = body.kv
        updated["kv"] = body.kv

    if body.ki is not None:
        if body.ki < 0:
            raise HTTPException(status_code=400, detail="ki must be >= 0")
        joint.set_dce_ki(body.ki)
        _dce_params[n]["ki"] = body.ki
        updated["ki"] = body.ki

    if body.kd is not None:
        if body.kd < 0:
            raise HTTPException(status_code=400, detail="kd must be >= 0")
        joint.set_dce_kd(body.kd)
        _dce_params[n]["kd"] = body.kd
        updated["kd"] = body.kd

    return {
        "ok": True,
        "joint": n,
        "updated": updated,
        "current": _dce_params[n],
    }


@app.post("/robot/joint/{n}/dce/reset")
def reset_dce(n: int):
    """
    Reset DCE parameters for joint n back to firmware defaults.
    Low-level mode required.
    """
    _check_low_level()
    if n < 1 or n > 6:
        raise HTTPException(status_code=400, detail="joint n must be 1–6")

    joint = get_joint(n)
    joint.set_dce_kp(_DCE_DEFAULTS["kp"])
    joint.set_dce_kv(_DCE_DEFAULTS["kv"])
    joint.set_dce_ki(_DCE_DEFAULTS["ki"])
    joint.set_dce_kd(_DCE_DEFAULTS["kd"])
    _dce_params[n] = dict(_DCE_DEFAULTS)

    return {"ok": True, "joint": n, "current": _dce_params[n]}


@app.get("/robot/angles")
def get_angles():
    r = get_robot().robot
    return {
        "j1": round(r.joint_1.angle, 3),
        "j2": round(r.joint_2.angle, 3),
        "j3": round(r.joint_3.angle, 3),
        "j4": round(r.joint_4.angle, 3),
        "j5": round(r.joint_5.angle, 3),
        "j6": round(r.joint_6.angle, 3),
    }


@app.get("/robot/info")
def get_info():
    d = get_robot()
    info = {
        "mode": "simulation" if SIM_MODE else "physical",
    }
    if SIM_MODE:
        info["serial_number"] = d.serial_number
        info["temperature"] = d.get_temperature()
        info["voltage"] = d.get_voltage()
        info["status"] = "running" if d.robot.enabled else "disabled"
    else:
        try:
            info["serial_number"] = str(d.serial_number)
        except Exception:
            info["serial_number"] = "unknown"
        try:
            info["temperature"] = d.get_temperature()
        except Exception:
            info["temperature"] = None
        try:
            info["voltage"] = d.get_voltage()
        except Exception:
            info["voltage"] = None
        info["status"] = "connected"
    return info


@app.get("/robot/status")
def get_status():
    """Quick health check: mode + connection status."""
    d = get_robot()
    return {
        "mode": "simulation" if SIM_MODE else "physical",
        "status": "connected" if not SIM_MODE else ("running" if d.robot.enabled else "disabled"),
    }


# ── Per-joint endpoints ───────────────────────────────────────────────────────
@app.get("/robot/joint/{n}/angle")
def joint_angle(n: int):
    return {"joint": n, "angle": round(get_joint(n).angle, 3)}


@app.post("/robot/joint/{n}/enable")
def joint_enable(n: int, body: EnableBody):
    get_joint(n).set_enable(body.enable)
    return {"ok": True}


@app.post("/robot/joint/{n}/set_position_with_time")
def joint_set_position(n: int, body: JointPosBody):
    """Low-level move. pos = target_degrees / 360 * reduction. Does NOT check angle limits."""
    get_joint(n).set_position_with_time(body.pos, body.vel)
    return {"ok": True}


@app.post("/robot/joint/{n}/do_calibration")
def joint_do_calibration(n: int):
    """Run encoder calibration. Motor will spin ~1 full revolution. Takes ~10 s."""
    get_joint(n).do_calibration()
    return {"ok": True}


@app.get("/robot/joint/{n}/temperature")
def joint_temperature(n: int):
    return {"joint": n, "temperature": get_joint(n).get_temperature()}


@app.post("/robot/joint/{n}/reboot")
def joint_reboot(n: int):
    get_joint(n).reboot()
    return {"ok": True}


@app.post("/robot/joint/{n}/erase_configs")
def joint_erase_configs(n: int):
    """Erase flash config on motor driver. Requires do_calibration() afterwards."""
    get_joint(n).erase_configs()
    return {"ok": True}


# ── Sequence player ──────────────────────────────────────────────────────────

# REST pose physical angles — motor encoders read 0 at this position
_REST_POSE = [0.0, -73.0, 180.0, 0.0, 0.0, 0.0]


def _physical_to_encoder(physical: list[float]) -> list[float]:
    """Convert physical joint angles (as used by move_j) to motor encoder angles."""
    return [physical[i] - _REST_POSE[i] for i in range(6)]


def _encoder_to_physical(encoder: list[float]) -> list[float]:
    """Convert motor encoder angles (0 = REST pose) back to physical joint angles."""
    return [encoder[i] + _REST_POSE[i] for i in range(6)]

class SequenceStep(BaseModel):
    # ── Move fields (for a single motion step) ───────────────────────────────
    move_j:  list[float] | None = None   # [j1,j2,j3,j4,j5,j6] degrees
    move_l:  list[float] | None = None   # [x,y,z,a,b,c] mm + degrees
    speed:   float | None = None         # per-step speed override (deg/s)
    hold:    float = 0.0                 # extra seconds to hold after settling
    comment: str   | None = None         # human-readable label (ignored by executor)

    # ── Block fields (for a repeating group of sub-steps) ────────────────────
    repeat: int              | None = None   # number of times to repeat sub-steps
    steps:  list['SequenceStep'] | None = None  # sub-steps to repeat

SequenceStep.model_rebuild()  # resolve forward reference


class PlaySequenceBody(BaseModel):
    speed:            float = 20.0   # default speed for all steps (deg/s)
    settle_threshold: float = 0.5    # degrees — "close enough" to target
    settle_timeout:   float = 15.0   # max seconds to wait for settling per step
    steps: list[SequenceStep]


def _read_angles(r) -> list[float]:
    """Read all 6 joint angles as a list."""
    return [
        r.joint_1.angle, r.joint_2.angle, r.joint_3.angle,
        r.joint_4.angle, r.joint_5.angle, r.joint_6.angle,
    ]


def _wait_for_settle(r, target: list[float], threshold: float, timeout: float,
                     check_stop: bool = False) -> float:
    """
    Poll joint angles until all joints are within `threshold` degrees of target,
    OR until `timeout` seconds have elapsed.
    If `check_stop=True`, also exits early when _stop_requested is set.
    Returns actual elapsed seconds.
    """
    start = time.monotonic()
    consecutive_settled = 0          # require 3 stable readings in a row
    while True:
        elapsed = time.monotonic() - start
        if elapsed >= timeout:
            break
        if check_stop and _stop_requested:
            break
        time.sleep(0.15)             # poll at ~6 Hz
        angles = _read_angles(r)
        all_close = all(abs(angles[i] - target[i]) <= threshold for i in range(6))
        if all_close:
            consecutive_settled += 1
            if consecutive_settled >= 3:
                break
        else:
            consecutive_settled = 0
    return time.monotonic() - start


@app.post("/robot/play_sequence")
def play_sequence(body: PlaySequenceBody):
    """
    Execute a list of moves in order, waiting for the arm to settle between steps.

    Steps can be single moves OR repeating blocks:
      {"move_j": [...], "hold": 0.5, "comment": "..."}   ← single move
      {"repeat": 5, "steps": [{...}, {...}]}               ← repeat block

    Blocks can be nested to any depth.
    Returns total wall-clock duration and a flat per-step log.

    Call POST /robot/stop at any time to abort mid-sequence and freeze in place.
    """
    return _run_sequence(body)


def _run_sequence(body: PlaySequenceBody) -> dict:
    """
    Core sequence execution engine. Used by both play_sequence and
    the named-sequence play endpoint.
    Clears _stop_requested on entry. Returns a result dict.
    """
    global _stop_requested
    _stop_requested = False   # clear any previous stop before we begin

    d = get_robot()
    r = d.robot

    r.set_enable(True)
    r.set_joint_speed(body.speed)

    t_start  = time.monotonic()
    step_log = []

    def execute_steps(steps: list, depth: int = 0):
        """Recursively execute steps, expanding repeat blocks."""
        for step in steps:
            # ── Check for stop request before every step ──────────────────
            if _stop_requested:
                raise _StopRequested()

            # ── Repeat block ──────────────────────────────────────────────
            if step.repeat is not None and step.steps is not None:
                for iteration in range(step.repeat):
                    execute_steps(step.steps, depth + 1)
                continue

            # ── Single move step ──────────────────────────────────────────
            step_start = time.monotonic()

            step_speed = step.speed if step.speed is not None else body.speed
            if step.speed is not None:
                r.set_joint_speed(step_speed)

            if step.move_j is not None:
                if len(step.move_j) != 6:
                    raise HTTPException(status_code=400,
                        detail=f"move_j must have exactly 6 values, got {len(step.move_j)}")
                j1, j2, j3, j4, j5, j6 = step.move_j
                r.move_j(j1, j2, j3, j4, j5, j6)
                target_enc = _physical_to_encoder(list(step.move_j))
                settle_s = _wait_for_settle(r, target_enc, body.settle_threshold, body.settle_timeout)

            elif step.move_l is not None:
                if len(step.move_l) != 6:
                    raise HTTPException(status_code=400,
                        detail=f"move_l must have exactly 6 values, got {len(step.move_l)}")
                x, y, z, a, b, c = step.move_l
                # Apply TCP offset and pass mm directly to firmware
                wx, wy, wz = _apply_tcp(x, y, z, a, b, c)
                r.move_l(wx, wy, wz, a, b, c)
                time.sleep(min(body.settle_timeout, 3.0))
                settle_s = min(body.settle_timeout, 3.0)
                time.sleep(min(body.settle_timeout, 3.0))
                settle_s = min(body.settle_timeout, 3.0)

            else:
                settle_s = 0.0  # pure hold/pause step

            if step.speed is not None:
                r.set_joint_speed(body.speed)

            if step.hold > 0:
                time.sleep(step.hold)

            angles_now = _read_angles(r)
            step_log.append({
                "comment":  step.comment,
                "angles":   [round(a, 2) for a in angles_now],
                "settle_s": round(settle_s, 2),
                "total_s":  round(time.monotonic() - step_start, 2),
            })

    try:
        execute_steps(body.steps)
    except _StopRequested:
        enc_now  = _read_angles(r)
        phys_now = _encoder_to_physical(enc_now)
        return {
            "ok":              False,
            "stopped":         True,
            "steps_completed": len(step_log),
            "frozen_at":       [round(a, 2) for a in phys_now],
            "total_s":         round(time.monotonic() - t_start, 2),
            "step_log":        step_log,
        }

    return {
        "ok":       True,
        "steps":    len(step_log),
        "total_s":  round(time.monotonic() - t_start, 2),
        "step_log": step_log,
    }


# ══════════════════════════════════════════════════════════════════════════════
# Named Sequence Library — CRUD + Play
# Sequences are stored in sequences.json and survive server restarts.
# ══════════════════════════════════════════════════════════════════════════════

@app.get("/sequences")
def list_sequences():
    """List all saved sequences (summary only — no steps)."""
    with _seq_lock:
        return {
            name: {
                "description": seq.get("description", ""),
                "step_count":  len(seq.get("steps", [])),
                "speed":       seq.get("speed"),
                "created_at":  seq.get("created_at"),
                "updated_at":  seq.get("updated_at"),
            }
            for name, seq in _sequences.items()
        }


@app.post("/sequences", status_code=201)
def create_sequence(body: CreateSequenceBody):
    """
    Create a new named sequence. Name must be lowercase, alphanumeric,
    hyphens or underscores, max 64 chars. Returns 409 if name already exists.
    """
    if not _SEQ_NAME_RE.match(body.name):
        raise HTTPException(status_code=400,
            detail="Name must be lowercase alphanumeric with hyphens/underscores (e.g. 'wave', 'pick-up')")
    with _seq_lock:
        if body.name in _sequences:
            raise HTTPException(status_code=409,
                detail=f"Sequence '{body.name}' already exists — use PUT to update")
        now = _now_iso()
        _sequences[body.name] = {
            "name":             body.name,
            "description":      body.description,
            "speed":            body.speed,
            "settle_threshold": body.settle_threshold,
            "settle_timeout":   body.settle_timeout,
            "steps":            body.steps,
            "created_at":       now,
            "updated_at":       now,
        }
        _save_sequences()
    return {"ok": True, "name": body.name, "created_at": now}


@app.get("/sequences/{name}")
def get_sequence(name: str):
    """Get the full definition of a named sequence including all steps."""
    with _seq_lock:
        if name not in _sequences:
            raise HTTPException(status_code=404, detail=f"Sequence '{name}' not found")
        return _sequences[name]


@app.put("/sequences/{name}")
def update_sequence(name: str, body: UpdateSequenceBody):
    """
    Update an existing sequence. Only provided fields are changed.
    Returns 404 if the sequence doesn't exist.
    """
    with _seq_lock:
        if name not in _sequences:
            raise HTTPException(status_code=404, detail=f"Sequence '{name}' not found")
        seq = _sequences[name]
        if body.description      is not None: seq["description"]      = body.description
        if body.speed            is not None: seq["speed"]            = body.speed
        if body.settle_threshold is not None: seq["settle_threshold"] = body.settle_threshold
        if body.settle_timeout   is not None: seq["settle_timeout"]   = body.settle_timeout
        if body.steps            is not None: seq["steps"]            = body.steps
        seq["updated_at"] = _now_iso()
        _save_sequences()
    return {"ok": True, "name": name, "updated_at": seq["updated_at"]}


@app.delete("/sequences/{name}")
def delete_sequence(name: str):
    """Delete a named sequence permanently."""
    with _seq_lock:
        if name not in _sequences:
            raise HTTPException(status_code=404, detail=f"Sequence '{name}' not found")
        del _sequences[name]
        _save_sequences()
    return {"ok": True, "deleted": name}


@app.post("/sequences/{name}/play")
def play_named_sequence(name: str, body: PlaySequenceOverrideBody = PlaySequenceOverrideBody()):
    """
    Play a saved sequence by name.
    Optionally override speed / settle_threshold / settle_timeout at call time
    without modifying the saved definition.
    """
    with _seq_lock:
        if name not in _sequences:
            raise HTTPException(status_code=404, detail=f"Sequence '{name}' not found")
        seq = dict(_sequences[name])   # shallow copy — safe to read outside lock

    # Build PlaySequenceBody from stored data, applying any runtime overrides
    play_body = PlaySequenceBody(
        speed            = body.speed            if body.speed            is not None else seq["speed"],
        settle_threshold = body.settle_threshold if body.settle_threshold is not None else seq["settle_threshold"],
        settle_timeout   = body.settle_timeout   if body.settle_timeout   is not None else seq["settle_timeout"],
        steps            = [SequenceStep(**s) if isinstance(s, dict) else s for s in seq["steps"]],
    )
    result = _run_sequence(play_body)
    result["sequence"] = name   # tag the result with the sequence name
    return result


# ── Sim-only: full state snapshot ─────────────────────────────────────────────@app.get("/robot/sim/state")
def sim_state():
    """
    Returns a full snapshot of the simulated robot state.
    Only meaningful in --sim mode; works in real mode too (reads live values).
    """
    r = get_robot().robot
    joints_out = {}
    for i, attr in enumerate(["joint_1", "joint_2", "joint_3",
                               "joint_4", "joint_5", "joint_6"]):
        j = getattr(r, attr)
        entry = {"angle": round(j.angle, 3)}
        if SIM_MODE:
            entry["target"] = round(j._target, 3)
            entry["moving"] = j.is_moving()
            entry["enabled"] = j.enabled
            entry["temperature"] = j.temperature
        joints_out[f"j{i + 1}"] = entry

    return {
        "sim": SIM_MODE,
        "enabled": r.enabled if SIM_MODE else None,
        "moving": r.is_moving() if SIM_MODE else None,
        "joints": joints_out,
    }


# ── WebSocket: live angle stream ──────────────────────────────────────────────
@app.websocket("/robot/stream")
async def stream_angles(ws: WebSocket):
    """
    Streams all 6 joint angles as JSON at ~10 Hz.

    Node.js example:
        const ws = new WebSocket('ws://localhost:3001/robot/stream');
        ws.on('message', (data) => console.log(JSON.parse(data)));
        // { j1: 0.01, j2: -0.02, j3: 0.00, j4: 0.00, j5: 0.00, j6: 0.00 }
    """
    await ws.accept()
    loop = asyncio.get_event_loop()
    try:
        while True:
            if dummy is None:
                await ws.send_json({"error": "Robot not connected"})
            elif SIM_MODE:
                # In sim mode: angles are updated in-memory — no blocking I/O needed
                r = dummy.robot
                await ws.send_json({
                    "j1": round(r.joint_1.angle, 3),
                    "j2": round(r.joint_2.angle, 3),
                    "j3": round(r.joint_3.angle, 3),
                    "j4": round(r.joint_4.angle, 3),
                    "j5": round(r.joint_5.angle, 3),
                    "j6": round(r.joint_6.angle, 3),
                    "sim": True,
                })
            else:
                r = dummy.robot
                angles = await loop.run_in_executor(None, lambda: {
                    "j1": r.joint_1.angle,
                    "j2": r.joint_2.angle,
                    "j3": r.joint_3.angle,
                    "j4": r.joint_4.angle,
                    "j5": r.joint_5.angle,
                    "j6": r.joint_6.angle,
                })
                await ws.send_json(angles)
            await asyncio.sleep(0.1)   # 10 Hz
    except WebSocketDisconnect:
        pass


# ══════════════════════════════════════════════════════════════════════════════
# Camera endpoints — Orbbec Astra Pro (RGB + Depth)
# Phase 2: ctypes persistent driver (~30ms/frame after first open)
# ══════════════════════════════════════════════════════════════════════════════

try:
    from camera_manager import CameraManager
    _camera_available = True
except ImportError:
    _camera_available = False


def _get_camera() -> "CameraManager":
    if not _camera_available:
        raise HTTPException(status_code=503, detail="camera_manager not available")
    return CameraManager.get()


@app.get("/camera/rgb", tags=["Camera"],
         summary="Capture RGB frame (JPEG)")
async def camera_rgb(warmup: float = 2.0):
    """
    Capture RGB frame from Astra Pro HD Camera (UVC).
    Returns JPEG. Query param `warmup`: seconds for auto-exposure (default 2.0).
    """
    import asyncio
    cm = _get_camera()
    try:
        data = await asyncio.get_event_loop().run_in_executor(
            None, lambda: cm.rgb.capture_bytes(warmup_secs=warmup)
        )
        return Response(content=data, media_type="image/jpeg")
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/camera/depth", tags=["Camera"],
         summary="Capture depth frame (PNG, ~30ms with persistent driver)")
async def camera_depth(width: int = 640, height: int = 480, fps: int = 30):
    """
    Capture depth frame via ctypes persistent driver.
    Returns colorized PNG (red=close, blue=far, auto p5-p95 range).
    Resolutions: 160×120, 320×240, 640×480 @ 30fps | 1280×1024 @ 7fps.
    First call ~0.5s (device open); subsequent calls ~30ms.
    """
    import asyncio
    cm = _get_camera()
    try:
        data, _ = await asyncio.get_event_loop().run_in_executor(
            None, lambda: cm.depth.capture_bytes(width, height, fps)
        )
        return Response(content=data, media_type="image/png")
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/camera/depth/hires", tags=["Camera"],
         summary="Capture hi-res depth frame 1280×1024 (PNG)")
async def camera_depth_hires():
    """
    Capture 1280×1024 depth frame @ 7fps.
    4× more pixels than 640×480. ~30ms capture after device open.
    Returns colorized PNG.
    """
    import asyncio
    cm = _get_camera()
    try:
        data, _ = await asyncio.get_event_loop().run_in_executor(
            None, cm.depth.capture_hires_bytes
        )
        return Response(content=data, media_type="image/png")
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/camera/depth/stats", tags=["Camera"],
         summary="Depth frame stats JSON only (~3ms, no PNG encoding)")
async def camera_depth_stats(width: int = 640, height: int = 480, fps: int = 30):
    """
    Capture depth frame and return stats as JSON. No PNG encoding.
    Extremely fast (~3ms) when device is already open.
    Returns: width, height, valid_pixels, valid_pct, depth_min/max/p5/p95/mean in mm.
    """
    import asyncio
    cm = _get_camera()
    try:
        stats = await asyncio.get_event_loop().run_in_executor(
            None, lambda: cm.depth.capture_stats(width, height, fps)
        )
        return stats
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/camera/snapshot", tags=["Camera"],
         summary="Capture RGB + depth simultaneously, return JSON")
async def camera_snapshot():
    """
    Capture RGB and depth in parallel.
    Saves to /tmp/snapshot_rgb.jpg and /tmp/snapshot_depth.png.
    Returns JSON with paths, depth stats, and UTC timestamp.
    """
    import asyncio
    from datetime import datetime, timezone

    cm = _get_camera()
    rgb_path   = "/tmp/snapshot_rgb.jpg"
    depth_path = "/tmp/snapshot_depth.png"

    async def capture_rgb():
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(
            None, lambda: cm.rgb.capture(rgb_path, warmup_secs=2.0)
        )

    async def capture_depth():
        loop = asyncio.get_event_loop()
        png_bytes, stats = await loop.run_in_executor(
            None, lambda: cm.depth.capture_bytes(640, 480, 30)
        )
        Path(depth_path).write_bytes(png_bytes)
        return stats

    try:
        rgb_ok, depth_stats = await asyncio.gather(capture_rgb(), capture_depth())
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

    return {
        "ok":          True,
        "rgb_path":    rgb_path,
        "depth_path":  depth_path,
        "rgb_ok":      rgb_ok,
        "depth_stats": depth_stats,
        "timestamp":   datetime.now(timezone.utc).isoformat(),
    }


@app.websocket("/camera/stream", name="Depth stream WebSocket")
async def camera_stream(websocket: WebSocket):
    """
    WebSocket depth frame stream.

    Connect to ws://127.0.0.1:3001/camera/stream
    Optionally send a JSON config message first:
        {"width": 640, "height": 480, "fps": 30, "format": "png"|"stats", "interval_ms": 100}

    Server pushes frames at requested interval:
    - format "png"  → binary PNG frame
    - format "stats" → JSON text with depth stats

    Send {"stop": true} to close gracefully.
    """
    import asyncio
    import json as _json

    await websocket.accept()
    cm = _get_camera()

    # Defaults
    width       = 640
    height      = 480
    fps         = 30
    fmt         = "png"     # "png" or "stats"
    interval_ms = 100       # ms between frames

    # Optional config from client
    try:
        raw = await asyncio.wait_for(websocket.receive_text(), timeout=1.0)
        cfg = _json.loads(raw)
        width       = cfg.get("width",       width)
        height      = cfg.get("height",      height)
        fps         = cfg.get("fps",         fps)
        fmt         = cfg.get("format",      fmt)
        interval_ms = cfg.get("interval_ms", interval_ms)
    except (asyncio.TimeoutError, Exception):
        pass  # no config sent — use defaults

    loop = asyncio.get_event_loop()
    try:
        while True:
            t0 = loop.time()

            try:
                if fmt == "stats":
                    stats = await loop.run_in_executor(
                        None, lambda: cm.depth.capture_stats(width, height, fps)
                    )
                    await websocket.send_text(_json.dumps(stats))
                else:
                    png_bytes, _ = await loop.run_in_executor(
                        None, lambda: cm.depth.capture_bytes(width, height, fps)
                    )
                    await websocket.send_bytes(png_bytes)
            except Exception as e:
                await websocket.send_text(_json.dumps({"error": str(e)}))
                break

            # Throttle to requested interval
            elapsed_ms = (loop.time() - t0) * 1000
            wait_ms    = max(0, interval_ms - elapsed_ms)
            if wait_ms > 0:
                await asyncio.sleep(wait_ms / 1000.0)

            # Check for stop message (non-blocking)
            try:
                msg = await asyncio.wait_for(websocket.receive_text(), timeout=0.001)
                if _json.loads(msg).get("stop"):
                    break
            except (asyncio.TimeoutError, Exception):
                pass

    except WebSocketDisconnect:
        pass
    finally:
        await websocket.close()


@app.get("/camera/depth/raw", tags=["Camera"],
         summary="Raw depth frame as uint16 binary (numpy-compatible)")
async def camera_depth_raw(width: int = 640, height: int = 480, fps: int = 30):
    """
    Return raw uint16 depth values as binary blob.
    First 8 bytes: [width(int32), height(int32)] as little-endian.
    Remaining bytes: H×W uint16 values in mm (row-major).
    Also sets X-Depth-Scale header = mm per unit (always 1.0 for Astra Pro).
    """
    import asyncio, struct
    import numpy as np
    cm = _get_camera()
    try:
        raw, W, H, scale = await asyncio.get_event_loop().run_in_executor(
            None, lambda: cm.depth.capture_raw(width, height, fps)
        )
        header = struct.pack('<ii', int(W), int(H))
        data = header + raw.astype('<u2').tobytes()
        return Response(content=data, media_type="application/octet-stream",
                        headers={"X-Depth-Scale": str(scale),
                                 "X-Depth-Width": str(W),
                                 "X-Depth-Height": str(H)})
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/camera/objects/detect", tags=["Camera"],
         summary="Detect elevated objects on the desk (JSON)")
async def camera_objects_detect(
    elevation_thresh_mm: float = 20.0,
    min_blob_pixels: int = 100,
    width: int = 640, height: int = 480
):
    """
    Detect 3D-printed objects on the desk using depth camera.

    Algorithm:
    1. Capture depth frame
    2. Estimate desk plane depth (p85 of valid pixels = background desk)
    3. Find pixels significantly closer than desk = elevated objects
    4. Label connected blobs, filter by size
    5. Return each blob's 3D centroid in camera frame + pixel location

    Returns JSON list of detected objects with:
      - id, pixels, center_uv, depth_mm, elevation_mm, cam_3d_mm [x,y,z]
    """
    import asyncio
    import numpy as np
    from scipy import ndimage as ndi

    cm = _get_camera()
    try:
        raw, W, H, scale = await asyncio.get_event_loop().run_in_executor(
            None, lambda: cm.depth.capture_raw(width, height, fps=30)
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

    frame = raw.reshape(H, W).astype(float) * scale

    # Intrinsics
    fx = fy = 570.34; cx, cy = W / 2.0 - 0.5, H / 2.0 - 0.5

    # Valid pixels (ignore sensor artifacts at very close range)
    valid = (frame > 100) & (frame < 1000)
    if valid.sum() < 1000:
        return {"ok": False, "detail": "insufficient valid depth pixels", "objects": []}

    # Desk plane: the dominant far surface (p85 of valid pixels)
    valid_depths = frame[valid]
    desk_depth = float(np.percentile(valid_depths, 85))

    # Object pixels: elevated above desk by threshold
    obj_mask = valid & (frame < (desk_depth - elevation_thresh_mm))

    if obj_mask.sum() < min_blob_pixels:
        return {"ok": True, "desk_depth_mm": round(desk_depth, 1),
                "n_objects": 0, "objects": [],
                "note": f"No objects found above desk (thresh={elevation_thresh_mm}mm)"}

    # Label blobs
    structure = np.ones((3, 3), dtype=int)
    labeled, n_blobs = ndi.label(obj_mask, structure=structure)

    objects = []
    for i in range(1, n_blobs + 1):
        blob = labeled == i
        n = int(blob.sum())
        if n < min_blob_pixels:
            continue
        vs, us = np.where(blob)
        depths_b = frame[blob]
        d_mean = float(depths_b.mean())
        d_min  = float(depths_b.min())
        u_c, v_c = float(us.mean()), float(vs.mean())
        x_cam = (u_c - cx) * d_mean / fx
        y_cam = (v_c - cy) * d_mean / fy
        elevation = desk_depth - d_mean
        objects.append({
            "id": i,
            "pixels": n,
            "center_uv": [round(u_c, 1), round(v_c, 1)],
            "depth_mm": round(d_mean, 1),
            "depth_top_mm": round(d_min, 1),
            "elevation_mm": round(elevation, 1),
            "cam_3d_mm": [round(x_cam, 1), round(y_cam, 1), round(d_mean, 1)],
        })

    # Sort by size (largest first)
    objects.sort(key=lambda o: -o["pixels"])

    return {
        "ok": True,
        "desk_depth_mm": round(desk_depth, 1),
        "n_objects": len(objects),
        "objects": objects,
        "frame_wh": [W, H],
        "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy},
    }


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import uvicorn
    # Strip --sim so uvicorn doesn't choke on an unknown arg
    args = [a for a in sys.argv[1:] if a != "--sim"]
    uvicorn.run(app, host="127.0.0.1", port=3001, log_level="info")
