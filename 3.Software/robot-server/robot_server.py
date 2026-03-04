#!/usr/bin/env python3
"""
Dummy X Robot Arm — FastAPI HTTP/WebSocket server

Bridges the Fibre USB protocol to a REST API callable from Node.js (or any HTTP client).

Usage:
    pip3 install fastapi uvicorn
    python3 robot_server.py          # starts on http://127.0.0.1:3001

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
from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# Add CLI-Tool to path so the existing fibre library can be imported as-is
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'CLI-Tool'))
import fibre

# ── Global robot connection ───────────────────────────────────────────────────
dummy = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    global dummy
    print("Connecting to robot via USB (timeout 10 s)...")
    loop = asyncio.get_event_loop()
    dummy = await loop.run_in_executor(
        None, lambda: fibre.find_any("usb", timeout=10)
    )
    if dummy is None:
        print("⚠  Robot not found. Endpoints will return 503 until connected.")
    else:
        print("✓  Robot connected.")
    yield
    dummy = None


app = FastAPI(title="Dummy Robot Arm API", version="1.0.0", lifespan=lifespan)
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
        "j1": r.joint_1.angle,
        "j2": r.joint_2.angle,
        "j3": r.joint_3.angle,
        "j4": r.joint_4.angle,
        "j5": r.joint_5.angle,
        "j6": r.joint_6.angle,
    }


@app.get("/robot/info")
def get_info():
    d = get_robot()
    return {
        "serial_number": d.serial_number,
        "temperature": d.get_temperature(),
        "voltage": d.get_voltage(),
    }


# ── Per-joint endpoints ───────────────────────────────────────────────────────
@app.get("/robot/joint/{n}/angle")
def joint_angle(n: int):
    return {"joint": n, "angle": get_joint(n).angle}


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
            await asyncio.sleep(0.1)  # 10 Hz
    except WebSocketDisconnect:
        pass


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=3001, log_level="info")

