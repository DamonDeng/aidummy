# Robot Server — Design Documentation

**Project:** Dummy X Robot Arm API Server  
**Location:** `3.Software/robot-server/`  
**Last updated:** 2026-03-11

---

## Overview

The robot server is a FastAPI HTTP/WebSocket server that bridges hardware (robot arm + depth camera) into a clean REST API. It runs locally on the host MacBook at `http://127.0.0.1:3001` and is intended to be called by AI agents, scripts, or any HTTP client.

```
AI Agent / Script
      │
      │  HTTP REST  (port 3001)
      ▼
 robot_server.py   (FastAPI)
      │
      ├── /robot/*     ──► Fibre USB ──► Robot Arm (STM32)
      │
      ├── /camera/*    ──► camera_manager.py
      │                         ├── RGBCamera   ──► imagesnap ──► Astra Pro HD Camera (UVC)
      │                         └── DepthCamera ──► depth_capture (C++) ──► Astra Pro Depth (OpenNI)
      │
      ├── /sequences/* ──► sequences.json (persistent named motion library)
      │
      └── /server/*    ──► mode switch (high_level / low_level)
```

---

## Module Breakdown

### `robot_server.py` — Main FastAPI application

The central server. Handles all HTTP routing and delegates to hardware layers.

**Design principles:**
- All blocking operations run in `asyncio.run_in_executor` to avoid blocking the event loop
- Sim mode (`--sim` flag) replaces hardware with a software mock — same API, same interface
- Stop flag pattern: `POST /robot/stop` sets a flag; all motion commands check it first
- Endpoints are grouped by tag: `Robot`, `Sequences`, `Camera`, `Server`

**Startup sequence:**
1. Parse `--sim` flag
2. If real hardware: connect via Fibre USB, configure joint parameters
3. Start FastAPI app on `127.0.0.1:3001`

---

### `camera_manager.py` — Camera abstraction layer

Provides a clean Python interface to both cameras. Uses subprocess bridging (Phase 1).

**Design decision: subprocess vs ctypes**

We chose subprocess bridging for Phase 1 because:
- The C++ binary (`depth_capture`) was already proven working during experimentation
- Zero risk of crashing the Python server (isolated process)
- Easy to debug (binary can be tested independently)
- Acceptable latency for current use case (~3–4s per capture)

Phase 2 (future): ctypes persistent driver — load `libOrbbecSDK.dylib` directly into Python, keep device open, enable streaming at 7–30fps.

**`RGBCamera`**
- Uses `imagesnap` subprocess (macOS CLI tool)
- Device: `"Astra Pro HD Camera"` (UVC — recognized as standard webcam)
- 2-second warmup for auto-exposure stabilization

**`DepthCamera`**
- Runs `depth_capture` binary via subprocess
- Sets `DYLD_LIBRARY_PATH=orbbec_sdk/` so the binary finds its dylibs
- Parses the **last non-empty stdout line** as JSON — SDK logs appear on earlier lines
- Singleton pattern: `CameraManager.get()`

---

### `depth_capture.cpp` — C++ depth capture tool

Standalone binary that captures one depth frame and saves a colorized PNG.

**Why C++ instead of Python?**
- OrbbecSDK only ships pre-built dylibs for C/C++ API on macOS
- `pyorbbecsdk` has no macOS builds (Linux/Windows only)
- The C++ approach proved working in experimentation; wrapping it is lower risk than reimplementing in Python

**Key implementation details:**
- Requests `OB_FORMAT_Y11` (NOT `OB_FORMAT_Y16`) — the SDK auto-unpacks Y11/Y12 to 16-bit mm values; requesting Y16 causes "No matched profile found" error
- Warms up 11 frames before capturing (first frames are often noisy or under-exposed)
- SDK logs print to stdout during `ob::Context` construction before any log-level call can suppress them — Python wrapper reads only the last stdout line
- Colormap: HSV H=0°(red) → H=240°(blue), auto-scaled to p5–p95 percentile range (avoids outlier clipping)

**Build:**
```bash
make depth_capture
# Requires: clang++, orbbec_sdk/include/, orbbec_sdk/libOrbbecSDK.dylib
```

---

### `orbbec_sdk/` — Orbbec SDK dylibs + headers

Pre-built macOS arm64/x86 universal binaries from OrbbecSDK v1.10.16.

**Why Astra Pro works despite not being "officially supported" on macOS:**
The macOS dylib contains an `OpenNIDeviceInfo.cpp` code path that handles legacy OpenNI protocol devices. The Astra Pro (PID 0x0403) speaks OpenNI protocol and is matched by this path. The official supported device list only covers newer UVC-based devices, but the OpenNI path was compiled in.

**Contents:**
- `libOrbbecSDK.dylib` — main SDK library
- `libob_usb.dylib` — custom USB communication (libusb-based)
- `liblive555.dylib` — network streaming (dependency)
- `include/` — C++ headers for `ob::Context`, `ob::Pipeline`, etc.

---

### `ik_solver.py` — Python IK solver

Analytical closed-form IK solver, ported from the STM32 firmware `DOF6Kinematic::SolveIK`.

**Why Python?**
- Enables pre-computing full paths before streaming to hardware
- 20,000 solves/sec — fast enough for real-time streaming
- Exact match to firmware: same Euler convention (Rz·Ry·Rx), same joint config selection

---

### `sequences.json` — Named motion library

Persistent JSON file storing named sequences (e.g., `elbow-wave`, `home-to-rest`).

**Structure:**
```json
{
  "sequence-name": {
    "name": "sequence-name",
    "description": "...",
    "speed": 20,
    "steps": [
      {"joints": [j1, j2, j3, j4, j5, j6], "duration_ms": 1000},
      ...
    ]
  }
}
```

---

## Camera API Design

### Endpoint design rationale

| Endpoint | Rationale |
|---|---|
| `GET /camera/rgb` | Simplest — just a JPEG image, usable by any client |
| `GET /camera/depth` | Standard 640×480, balance of speed and detail |
| `GET /camera/depth/hires` | 1280×1024 for when precision matters (object detection) |
| `GET /camera/depth/stats` | Stats-only when image not needed (faster, less bandwidth) |
| `GET /camera/snapshot` | Parallel RGB+depth capture for synchronized frames |

**All endpoints are async** — blocking work runs in thread pool via `run_in_executor`.

**Image format choices:**
- RGB → JPEG (smaller, lossy acceptable for visual reference)
- Depth → PNG (lossless required — depth values must be exact)

### Depth colormap

Red→Yellow→Green→Cyan→Blue (HSV 0°→240°):
- **Red** = closest objects (near camera)
- **Blue** = furthest objects (far from camera)
- **Dark gray** = no data (IR dropout on dark/shiny surfaces, or out of range)
- **Auto-scaled** to p5–p95 of valid pixel range — avoids single bright/dark pixel washing out the color range

### Known limitations
1. **~3–4s latency per depth capture** — subprocess startup + 11-frame warmup. Acceptable for single captures; not for streaming.
2. **USB cooldown needed** — rapid successive depth captures can time out; the device needs ~5s between calls.
3. **Dark surface dropout** — robot arm body mostly missing from depth map (black anodized aluminum + black housing). Normal behavior for structured-light cameras.
4. **No depth-to-color alignment** — depth and RGB frames are not spatially aligned (D2C). The depth module and RGB lens are ~5cm apart on the device. Alignment requires OrbbecSDK D2C pipeline (future work).

---

## Robot Arm API Design

### Joint space vs Cartesian space

The arm exposes both:
- **`/robot/move_j`** — direct joint angles (degrees). Fast, predictable, no IK required.
- **`/robot/move_l`** — Cartesian target (x, y, z in mm + a, b, c Euler angles). IK solved server-side in Python (`ik_solver.py`).
- **`/robot/move_l_path`** — pre-computes IK for a full path in Python, streams MoveJ at configurable interval. Enables smooth Cartesian motion.

### Stop flag pattern

`POST /robot/stop` sets a server-side flag. All motion endpoints check this flag and reject commands if set. The flag is cleared by: `/robot/homing`, `/robot/resting`, `/robot/z_home`, `/robot/play_sequence`.

This prevents motion commands from firing after an emergency stop without explicitly resetting to a known safe state.

### Named sequences

Sequences are stored in `sequences.json` and managed via CRUD endpoints. A sequence is a list of joint-angle waypoints with timing. Playing a sequence moves through each waypoint at the configured speed.

---

## Future Roadmap

### Phase 2 Camera: ctypes persistent driver
Replace subprocess with direct `libOrbbecSDK.dylib` loading via Python ctypes:
- Device stays open between API calls → eliminates 3–4s startup latency
- Enables streaming via WebSocket at up to 7fps (1280×1024) or 30fps (640×480)
- New endpoint: `WS /camera/stream`

### Object Detection
`POST /camera/objects/detect`:
1. Capture 1280×1024 depth frame
2. Estimate desk plane (RANSAC or mean of flat region)
3. Find pixels elevated above plane by threshold (e.g. >10mm)
4. Connected component labeling → individual objects
5. Per-object: compute centroid (u, v) → project to 3D (X, Y, Z) using camera intrinsics
6. Return: `[{"id": 0, "x_mm": 245, "y_mm": 180, "z_mm": 142, "w_mm": 45, "h_mm": 30}]`

### Pick-and-Place Loop
```
GET /camera/objects/detect
    → object (X, Y, Z) in camera frame
    → transform to robot arm base frame (requires calibration)
    → POST /robot/move_l  (approach position above object)
    → activate gripper
    → POST /robot/move_l  (lift)
    → POST /robot/move_l  (place position)
    → release gripper
```

### Camera-to-Robot Calibration
Hand-eye calibration: move arm to known positions, observe tip in camera frame, solve for rigid transform between camera frame and robot base frame.
