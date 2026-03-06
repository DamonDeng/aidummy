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
from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

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

_HOME_POSE = [0.0,   0.0, 90.0, 0.0, 0.0, 0.0]
_REST_POSE = [0.0, -73.0, 180.0, 0.0, 0.0, 0.0]
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
_sim_task = None   # background simulation loop handle


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


class SpeedBody(BaseModel):
    speed: float


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
def homing():
    get_robot().robot.homing()
    return {"ok": True}


@app.post("/robot/resting")
def resting():
    get_robot().robot.resting()
    return {"ok": True}


@app.post("/robot/move_j")
def move_j(body: MoveJBody):
    get_robot().robot.move_j(body.j1, body.j2, body.j3, body.j4, body.j5, body.j6)
    return {"ok": True}


@app.post("/robot/move_l")
def move_l(body: MoveLBody):
    get_robot().robot.move_l(body.x, body.y, body.z, body.a, body.b, body.c)
    return {"ok": True}


@app.post("/robot/set_joint_speed")
def set_joint_speed(body: SpeedBody):
    get_robot().robot.set_joint_speed(body.speed)
    return {"ok": True}


@app.post("/robot/calibrate_home_offset")
def calibrate_home_offset():
    get_robot().robot.calibrate_home_offset()
    return {"ok": True}


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


def _wait_for_settle(r, target: list[float], threshold: float, timeout: float) -> float:
    """
    Poll joint angles until all joints are within `threshold` degrees of target,
    OR until `timeout` seconds have elapsed.
    Returns actual elapsed seconds.
    """
    start = time.monotonic()
    consecutive_settled = 0          # require 3 stable readings in a row
    while True:
        elapsed = time.monotonic() - start
        if elapsed >= timeout:
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
    """
    d = get_robot()
    r = d.robot

    r.set_enable(True)
    r.set_joint_speed(body.speed)

    t_start  = time.monotonic()
    step_log = []

    def execute_steps(steps: list, depth: int = 0):
        """Recursively execute steps, expanding repeat blocks."""
        for step in steps:
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
                r.move_l(*step.move_l)
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

    execute_steps(body.steps)

    return {
        "ok":       True,
        "steps":    len(step_log),
        "total_s":  round(time.monotonic() - t_start, 2),
        "step_log": step_log,
    }


# ── Sim-only: full state snapshot ─────────────────────────────────────────────
@app.get("/robot/sim/state")
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


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import uvicorn
    # Strip --sim so uvicorn doesn't choke on an unknown arg
    args = [a for a in sys.argv[1:] if a != "--sim"]
    uvicorn.run(app, host="127.0.0.1", port=3001, log_level="info")
