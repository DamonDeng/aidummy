# General Vision System Plan
*Written: 2026-03-12*

## Goal

Build a general-purpose, environment-agnostic pick-and-place system that:
- Works anywhere (no desk-specific calibration, no environment-specific setup)
- Uses Large Vision Models for object understanding (natural language queries)
- Self-calibrates using the arm itself as the reference target
- Looks like a polished product, not a lab prototype

---

## Architecture Overview

```
User command: "pick up the yellow cup"
        │
        ▼
[Scene Understanding Module]
  RGB frame → VLM (Claude / GPT-4o / Gemini)
  "locate the yellow cup" → pixel bounding box (x, y)
        │
        ▼
[3D Grounding Module]
  pixel (x, y) → depth lookup → 3D point in camera frame
  + camera intrinsics
        │
        ▼
[Coordinate Transform]
  camera frame → robot base frame
  (via stored hand-eye calibration matrix)
        │
        ▼
[Motion Planning]
  3D target position → IK solver → joint angles
  + pre-grasp approach + grasp + lift
        │
        ▼
Robot executes
```

---

## Module 1: Self-Calibration (`arm_calibration.py`)

### Concept
Use the arm tip itself as the calibration target — no markers, no special hardware.
The arm's FK always gives exact 3D tip position. Move to N known positions, detect
the tip in the image via VLM, then solve for camera pose with PnP.

### Process
1. Move arm tip to 8 positions spanning the workspace (FK gives us ground-truth 3D XYZ)
2. At each position: capture RGB photo
3. Ask VLM: *"What are the pixel coordinates (x, y) of the robot arm's tip in this image?"*
4. Collect 8× (3D_point, 2D_pixel) pairs
5. Run cv2.solvePnP() → camera rotation + translation relative to arm base
6. Store as `calibration/camera_to_robot.json`

### Trigger
- API endpoint: `POST /robot/calibrate`
- Re-run only when camera is physically moved

### Key design decisions
- Arm tip can have a **distinctive colored cap** (coral, gold, etc.) — elegant, not a marker
- 8 positions chosen to span the full workspace volume (varying J1, J2, J3)
- VLM provides robustness: works even if arm partially overlaps background
- Fallback: manual pixel annotation if VLM confidence is low

---

## Module 2: Scene Understanding (`scene_understanding.py`)

### Concept
VLM (Claude Vision / GPT-4o) handles all object recognition.
No trained models, no color thresholds, no environment-specific logic.
Generalizes to any object describable in natural language.

### API
```
GET /objects/find?query=yellow+cup
→ {"object": "yellow cup", "pixel_x": 342, "pixel_y": 218, "confidence": 0.92,
   "x_mm": 145.2, "y_mm": -82.3, "z_mm": 48.7}
```

### Process
1. Capture RGB + depth frame
2. Send RGB to VLM: *"Find the {query}. Return its center pixel as JSON: {x, y}"*
3. Look up depth at (x, y) → distance from camera
4. Apply camera intrinsics: pixel → ray → 3D point in camera frame
5. Apply hand-eye transform → 3D in robot frame
6. Return full result

### VLM prompt design
- Structured output request (JSON mode)
- Include image + concise instruction
- Confidence scoring via follow-up: "How confident are you? 0-1"
- Fallback: ask for bounding box if point is uncertain, use center

---

## Module 3: Coordinate Transform

### Camera Intrinsics (Astra Pro)
- Depth camera: known from OrbbecSDK calibration data
- RGB camera: can be estimated via OpenCV calibration checkerboard (one-time)
- Or: use Astra Pro factory intrinsics (640×480: fx≈570, fy≈570, cx=320, cy=240)

### Hand-Eye Transform
- 4×4 homogeneous matrix: `T_robot_camera`
- Solved by `arm_calibration.py` via PnP
- Stored in `calibration/camera_to_robot.json`
- Format: rotation (3×3) + translation (3×1), in mm

### Pixel → 3D
```python
d = depth_frame[y, x]  # mm
X = (x - cx) * d / fx
Y = (y - cy) * d / fy
Z = d
point_camera = [X, Y, Z, 1]
point_robot = T_robot_camera @ point_camera
```

---

## Module 4: Motion Planning (Existing + Extensions)

### Existing
- IK solver (`ik_solver.py`) — fast, analytical, 0.05ms per solve ✓
- `/robot/move_j`, `/robot/move_l_path` — working ✓

### New: Grasp Planning
- Pre-grasp approach: come from above at +100mm Z offset, then descend
- Orientation: J4/J5 to align gripper with object's longest axis (from VLM shape description)
- Contact detection: monitor joint torques or use depth delta

---

## Hardware Design Notes (for Damon to work on)

### Philosophy: Look Designed, Not Experimental

Every modification should look like it was always intended — not added for testing.

### Ideas
1. **Arm tip cap** — small colored sphere/dome, 3D-printable
   - Snap-on design over existing tool tip
   - Distinctive color (coral #FF6B6B, or metallic gold, or electric blue)
   - ~20–25mm diameter — large enough to detect, small enough to look natural
   - This doubles as a calibration reference + visual anchor for VLM

2. **Custom color scheme**
   - All industrial robots have a signature color — KUKA orange, ABB mustard, Fanuc yellow
   - Suggestion: matte black body + accent color at joints (orange rings, or white bands)
   - Or: two-tone matte white + color accents (cleaner, camera-friendly)

3. **Link surface treatment**
   - Forearm link (J3→J5): matte finish helps depth camera (less IR specular)
   - Base + shoulder: could be different color = natural "design accent" + helps VLM distinguish arm segments
   - Avoid pure black on surfaces that face the depth camera (IR dropout)

4. **Logo / identity**
   - A custom nameplate or laser-etched logo on the base
   - Makes it look like a product, not a prototype

---

## Implementation Roadmap

### Phase 1: Prove VLM Tip Detection (Current)
- [x] Concept validated (this document)
- [ ] Move arm to 2 positions, capture photos
- [ ] Test VLM tip pixel detection accuracy
- [ ] Validate: returned (x, y) matches visible tip location

### Phase 2: Full Self-Calibration
- [ ] Build `arm_calibration.py` with 8-pose routine
- [ ] Integrate cv2.solvePnP
- [ ] `POST /robot/calibrate` endpoint
- [ ] Store + load calibration matrix

### Phase 3: Scene Understanding
- [ ] Build `scene_understanding.py`
- [ ] `GET /objects/find?query=...` endpoint
- [ ] Test with various objects and environments

### Phase 4: End-to-End Pick-and-Place
- [ ] Connect all modules
- [ ] Test in original desk setup
- [ ] Test in a new environment (prove generalization)

---

## Files

| File | Location | Status |
|------|----------|--------|
| This plan | `docs/general-vision-plan.md` | ✓ |
| IK solver | `3.Software/robot-server/ik_solver.py` | ✓ existing |
| Robot server | `3.Software/robot-server/robot_server.py` | ✓ existing |
| Calibration module | `3.Software/robot-server/arm_calibration.py` | planned |
| Scene understanding | `3.Software/robot-server/scene_understanding.py` | planned |
| Calibration data | `3.Software/robot-server/calibration/camera_to_robot.json` | planned |

---

*This system makes the arm general-purpose. The arm + camera become a reusable platform
that can be placed anywhere and told what to do in plain language.*
