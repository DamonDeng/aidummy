# EXP-013 — Orbbec Astra Pro Depth Camera Integration

**Date:** 2026-03-11  
**Goal:** Integrate Orbbec Astra Pro depth camera into the robot arm system — get RGB + depth working on macOS, expose via REST API

---

## Motivation

To enable robot arm pick-and-place, we need visual sensing of the workspace:
- Know where objects are on the desk (X, Y, Z in mm)
- Detect which objects are present and their positions
- Eventually feed object coordinates directly into `/robot/move_l` for autonomous grasping

The Astra Pro provides both RGB (for visual reference) and structured-light depth (for 3D coordinate extraction).

---

## Hardware Setup

- **Device:** Orbbec Astra Pro (structured light depth camera)
- **USB IDs:**
  - Depth module: VID=0x2bc5, PID=0x0403 (OpenNI legacy protocol)
  - RGB module: VID=0x2bc5, PID=0x0501 (UVC — standard camera)
- **Firmware:** RD108C-009, Serial: 17122313469
- **Connection:** USB 2.0 via Genesys Logic hub (desk dock)
- **Position:** Elevated above desk, angled downward toward arm workspace

---

## Phase 1: Driver Discovery

### Problem
Orbbec officially states macOS only supports newer devices (Gemini 2, Astra 2, etc.) via OrbbecSDK v1. The Astra Pro uses the legacy OpenNI protocol, which has no official macOS SDK.

### Key Discovery
OrbbecSDK v1.10.16 macOS `libOrbbecSDK.dylib` contains a hidden `OpenNIDeviceInfo.cpp` code path that speaks the OpenNI legacy protocol — despite not being in the official supported device list.

**Test result:**
```
[OpenNIDeviceInfo.cpp:186] New openni device matched.
Name: Astra Pro, PID: 0x0403, SN/ID: , Connection: USB2.0
FW: RD108C-009, Serial: 17122313469
```

### SDK Source
- Downloaded: `OrbbecSDK_C_C++_v1.10.16_20241021_c0329e3_macos_arm64_x86.zip` from GitHub releases
- Stored at: `3.Software/robot-server/orbbec_sdk/`
- Repos cloned: `~/Desktop/workspace/OpenNI_SDK/`, `~/Desktop/workspace/OrbbecSDK/`

---

## Phase 2: Depth Stream Verification

### First depth frames (C++ test program)
```
Frame 1: 160x120 center depth=76mm
Frame 2: 160x120 center depth=76mm
...
```
Confirmed: depth data flowing on macOS arm64, **no sudo required**.

### Supported depth profiles
| Resolution | FPS | Format | Notes |
|---|---|---|---|
| 160×120 | 30fps | Y11/Y12 | Fast preview |
| 320×240 | 30fps | Y11/Y12 | Medium |
| 640×480 | 30fps | Y11/Y12 | Standard ✅ |
| **1280×1024** | **7fps** | **Y11/Y12** | **Hi-res ✅** |

**Important:** Must request `OB_FORMAT_Y11` (not `OB_FORMAT_Y16`) — SDK auto-unpacks to 16-bit mm values.

### SDK log issue
The SDK prints INFO/WARNING logs to stdout during `ob::Context` construction, before any log-level setting takes effect. These appear before the JSON output. **Workaround:** Python wrapper parses the **last non-empty line** of stdout as the JSON, ignoring SDK log lines above it.

---

## Phase 3: Depth Visualization

### Colormap
- Red (close) → Yellow → Green → Cyan → Blue (far)
- Dark gray (30,30,30) = no-data pixels (IR dropout)
- Auto-range: computed from p5–p95 percentile of valid depths (avoids outlier clipping)

### Camera positioning observations
| Position | Valid pixels | Depth range | Quality |
|---|---|---|---|
| Too close (first test) | 47% | 44–346mm | Poor — below min range |
| Mid distance | 73–75% | 62–497mm | Improving |
| Elevated + angled down | **82%** | 74–572mm | Good ✅ |

### Astra Pro depth specs
- Min reliable range: ~200mm
- Max range: ~3000mm (best accuracy 600–1500mm)
- Dark/black surfaces → IR dropout (expected — structured light limitation)
- White/light surfaces → excellent readings

### Physical setup (final)
- Camera elevated above desk on improvised mount
- Angled downward toward arm workspace
- White paper on desk as pickup zone reference surface
- Two white 3D-printed objects (cube + ring) as test targets
- Both objects clearly distinguishable at 640×480 and 1280×1024

---

## Phase 4: API Server Integration

### Files created
```
3.Software/robot-server/
├── orbbec_sdk/               ← SDK dylibs + headers
│   ├── libOrbbecSDK.dylib
│   ├── libOrbbecSDK.1.10.dylib
│   ├── libOrbbecSDK.1.10.16.dylib
│   ├── libob_usb.dylib
│   ├── liblive555.dylib
│   └── include/              ← C++ headers
├── depth_capture.cpp         ← C++ capture tool
├── stb_image_write.h         ← PNG encoder (header-only)
├── Makefile                  ← Build system
└── camera_manager.py         ← Python wrapper module
```

### depth_capture binary
- Captures one depth frame, writes colorized PNG to given path
- Prints stats JSON to stdout (last line, after any SDK log noise)
- Usage: `DYLD_LIBRARY_PATH=orbbec_sdk ./depth_capture <out.png> [w] [h] [fps]`
- Warms up 11 frames before capture
- Compile: `make depth_capture`

### camera_manager.py
- `RGBCamera.capture_bytes()` → JPEG bytes via imagesnap subprocess
- `DepthCamera.capture(path, w, h, fps)` → stats dict, saves PNG
- `DepthCamera.capture_bytes(w, h, fps)` → (PNG bytes, stats dict)
- `DepthCamera.capture_hires_bytes()` → 1280×1024 @ 7fps
- `CameraManager.get()` → singleton instance

### New REST endpoints (added to robot_server.py)
| Method | Endpoint | Returns | Latency |
|---|---|---|---|
| GET | `/camera/rgb` | JPEG image | ~2–3s |
| GET | `/camera/depth` | PNG colorized | ~3–4s |
| GET | `/camera/depth/hires` | PNG 1280×1024 | ~5–6s |
| GET | `/camera/depth/stats` | JSON stats only | ~3–4s |
| GET | `/camera/snapshot` | JSON (RGB+depth paths+stats) | ~4–5s parallel |

### Verified API results
```json
// GET /camera/depth/stats
{
  "ok": true,
  "width": 640, "height": 480,
  "valid_pixels": 252148, "total_pixels": 307200,
  "valid_pct": 82,
  "depth_min_mm": 74, "depth_max_mm": 554,
  "depth_p5_mm": 80, "depth_p95_mm": 464,
  "depth_mean_mm": 220
}
```
- `GET /camera/rgb` → JPEG 1280×720, ~216KB ✅
- `GET /camera/depth` → PNG 640×480, ~78KB ✅

### Known issue
`/camera/snapshot` can time out if the depth sensor is called too rapidly in succession (USB device needs ~5s cooldown between calls). Non-critical for current use.

---

## Key Learnings

1. **OrbbecSDK v1 macOS works with Astra Pro** despite not being officially listed — the OpenNI protocol path is compiled in.
2. **Request OB_FORMAT_Y11, not OB_FORMAT_Y16** — the SDK unpacks automatically; requesting Y16 causes "No matched profile found" error.
3. **SDK logs are stdout-bound** during Context construction — parse last line for JSON, not full stdout.
4. **Black surfaces = IR dropout** — robot arm body always has holes in depth map. This is OK for workspace object detection (we care about objects, not the arm).
5. **1280×1024 @ 7fps** is the maximum resolution — 4× more pixels than 640×480, good for precise object centroid detection.
6. **p5–p95 auto-range colormap** gives much better visualization than fixed range, especially when scene depth varies widely.

---

## Next Steps

- [ ] Object detection: desk plane estimation + elevated pixel clustering → object centroids (X, Y, Z) in mm
- [ ] Coordinate transform: camera frame → robot arm base frame (calibration)
- [ ] Pick-and-place loop: `/camera/objects/detect` → `/robot/move_l` → grasp
- [ ] Camera mount: purchase proper camera support arm for stable positioning
- [ ] Optional: Phase 2 ctypes persistent driver (no subprocess overhead, enables streaming)
