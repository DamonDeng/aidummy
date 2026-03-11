# EXP-017: Visual Approach Attempt — Touch Probe on Yellow Object

**Date:** 2026-03-11 (evening)
**Status:** ❌ FAILED
**Objective:** Position arm tip directly above (then onto) the yellow 3D-printed gear object using depth + RGB visual feedback, as a first step toward camera-robot hand-eye calibration.

---

## Setup

- Server: FastAPI at `http://127.0.0.1:3001`, physical mode
- Camera: Orbbec Astra Pro (RGB 1280×720, depth 640×480)
- 3 objects on desk: 1 yellow gear (3D printed), 2 white (cylinder + square box)
- Camera positioned at left side of arm's workspace, looking rightward across the desk

---

## What Was Attempted

### Phase 1: J1 Alignment via Depth Background Subtraction
- Moved arm to rest, captured background depth frame
- Moved arm to scout position [J1=0, J2=10, J3=85], captured foreground
- Computed diff to find arm pixels → only **17 pixels detected**
- Root cause: arm tip and objects are both at ~208mm from camera → same depth plane → subtraction fails

### Phase 2: Coarse J1 Sweep (-30° to +30°)
- Swept J1 from -30° to +30° comparing arm centroid u-pixel to object u-pixel
- Found "best" J1=-30° (arm centroid 7px from object centroid in depth frame)
- **FLAW:** Matching arm body centroid to object centroid doesn't mean tip is above object — the arm body spans many pixels, centroid location is misleading

### Phase 3: Descent Sequence [J1=-30, J2=30→43, J3=70→62]
- Descended through 5 steps, FK tip went from z=254mm → z=217mm
- Depth at target pixel stabilized at ~166mm → code triggered "stable contact" at J2=43
- **FLAW:** z=217mm is still far too high above desk objects (~50mm tall). "Depth stable" was the arm body visible in the depth frame, not the tip touching anything
- **FLAW:** J3=62-70° is the wrong elbow configuration — keeps arm tip HIGH

### Phase 4: Visual Sweep (RGB photos at multiple J1 angles)
After inspecting the descent_43.jpg photo and confirming tip was nowhere near yellow object:
- Redesigned approach: use J2=25, J3=130 (which gives z=94mm — low enough to be visible over objects)
- Swept J1 from -30° to -45° in steps, captured RGB photo at each, sent to Feishu
- User feedback:
  - J1=-30: tip near white square
  - J1=-35: tip between white square and white cylinder
  - J1=-40: tip almost on white cylinder, slightly toward yellow gear
  - J1=-41, -43, -45: fine sweep near yellow gear range
- Arm was getting directionally closer but **never confirmed directly above yellow gear**
- Experiment stopped here by user decision

---

## Root Causes of Failure

### 1. Wrong elbow configuration (primary failure)
- Descent used J3=60-70° → tip stays at z=217-254mm (way too high)
- Correct for desk-level: J3=115-130° with corresponding J2=25-45°
- **Lesson:** `z_tip ≈ 50–100mm` requires J3 ≈ 120–130°, not J3=60-70°

### 2. Depth background subtraction fails when arm ≈ objects depth
- Arm tip and objects both at ~208mm from camera
- Diff threshold of 25mm is too small to separate them
- Method only works if arm is at significantly different depth than objects

### 3. No ground-truth camera-robot transform
- Rough estimate (t≈464, 78, 171mm) has 91mm residuals — not usable for blind positioning
- All approach attempts were open-loop guesses without reliable coordinate mapping
- Visual servoing requires either: (a) good transform, or (b) direct visual confirmation at each step

### 4. RGB-to-depth pixel mapping error
- Assumed simple sx=2.0, sy=1.5 scaling from RGB (1280×720) to depth (640×480)
- Actually: depth and RGB sensors have different optical centers AND different baselines
- Color analysis using scaled coordinates mapped to wrong physical location

### 5. Depth "stable contact" detection was unreliable
- Detecting arm body in depth frame, not arm tip
- Need a more reliable contact detection method (e.g. torque sensing, force feedback, or IR marker on tip)

---

## What Actually Worked

- **J3 geometry insight confirmed:** For desk-level reach, J3 must be ~120-130°
- **Object lateral position estimated:** Yellow gear is between J1=-43° and J1=-45° laterally
- **Direction confirmed:** Negative J1 = arm swings toward objects (they are to the right/negative-Y side)
- **RGB visual sweep** is a valid manual-confirmation method for J1 alignment

---

## FK Positions at Key Poses
| Pose | J1 | J2 | J3 | FK tip (robot mm) | Notes |
|---|---|---|---|---|---|
| Near white square | -30 | 25 | 130 | (193,-111,94) | |
| Between sq & cyl | -35 | 25 | 130 | (183,-128,94) | |
| Near white cylinder | -40 | 25 | 130 | (171,-143,94) | closest to yellow gear in sweep |
| Fine sweep | -41 | 25 | 130 | (168,-146,94) | |
| Fine sweep | -43 | 25 | 130 | (163,-152,94) | likely closest to yellow gear |
| Fine sweep | -45 | 25 | 130 | (158,-158,94) | |

---

## Proposed Better Approach (for next experiment)

The "put tip on top of object" approach is fundamentally limited without a camera-robot transform. Better alternatives:

### Option A: Structured calibration first
1. Use a known reference target (e.g. tape cross on desk at known position)
2. Move arm to that exact position using FK, record pixel in RGB frame
3. Repeat for 3+ points → solve camera-robot transform properly
4. Then use transform for object positioning

### Option B: Top-down camera
- Mount camera directly above workspace (top-down view)
- Object x,y maps directly to camera u,v (no perspective distortion)
- Much simpler geometry for pick-and-place

### Option C: Wrist-mounted camera
- Camera on the arm itself sees the object in consistent frame
- Visual servoing becomes straightforward: center object in camera → descend

### Option D: Joint-by-joint manual teach
- User manually jogs arm to each object using Feishu commands
- Record joint angles as calibration points
- Use those as pick positions directly (no vision needed)

---

## Conclusion

The experiment correctly identified object lateral position range (J1 ≈ -43° to -45° for yellow gear) but failed to achieve tip-on-object contact due to wrong elbow configuration in early attempts and insufficient camera-robot calibration for reliable open-loop positioning. The approach is not the right method without a proper hand-eye calibration transform. Stopping here is the correct decision.

**Time spent:** ~2.5 hours
**Commits this session:** None (endpoints added but not committed)
