# EXP-009 — move_l Cartesian Control Exploration

**Date:** 2026-03-07  
**Goal:** Understand and validate the `move_l` (Cartesian) command on the robot arm  

---

## Summary

Successfully validated `move_l` Cartesian control after debugging two critical issues.  
Demonstrated orientation-controlled wrist movement (b: 0° ↔ 90°) at a fixed tip position.

---

## Bugs Found & Fixed

### Bug 1 — IK return value ignored
**Original code:**
```python
def move_l(body: MoveLBody):
    get_robot().robot.move_l(body.x, body.y, body.z, body.a, body.b, body.c)
    return {"ok": True}
```
**Problem:** Firmware's `MoveL` returns `false` when IK finds no valid solution. Server ignored this and always returned `{"ok": true}`.  
**Fix:** Capture return value and return `{"ok": false, "detail": "IK failed..."}` on failure.

### Bug 2 — Wrong unit assumption (double-division)
**Problem:** Assumed firmware IK expected metres, added `/1000` division in server.  
**Root cause found in firmware (`6dof_kinematic.cpp` line ~210):**
```cpp
P06[0] = _inputPose6D.X / 1000.0f;   // IK divides by 1000 internally!
P06[1] = _inputPose6D.Y / 1000.0f;
P06[2] = _inputPose6D.Z / 1000.0f;
```
The IK solver **already converts mm → metres internally**. Adding `/1000` in the server caused double-division: 200mm → 0.2m → firmware → 0.0002m = 0.2mm. IK found no solutions for 0.2mm (way too small).  
**Fix:** Remove the `/1000` from server. Pass mm directly.

**Unit convention (FINAL, CONFIRMED):**
- API input: **millimetres** (x, y, z)
- Server passes: **millimetres** to firmware
- Firmware IK: divides by 1000 internally → works in **metres** (matching DH parameters)

---

## FK Results (corrected, exact firmware formula)

| Pose | Joints | x (mm) | y (mm) | z (mm) | a (°) | b (°) | c (°) |
|---|---|---|---|---|---|---|---|
| REST | [0,-73,180,0,0,0] | 89.4 | 0 | 146.7 | — | — | — |
| HOME | [0,0,90,0,0,0] | 222.0 | 0 | 307.0 | 0 | 0 | 0 |
| Z-HOME | [0,-45,140,0,0,0] | 122.58 | 0 | 247.74 | 180 | 85 | 180 |

---

## Orientation Discovery

At Z-home position (122.58, 0, 247.74):

| b value | Result | Notes |
|---|---|---|
| 85° | ✅ | Natural Z-home orientation |
| 90° | ✅ | Tip pointing up |
| 0° | ✅ | Alternative configuration |
| 180° | ❌ IK failed | Not reachable at this position |

**Key insight:** Not all (x,y,z) + (a,b,c) combinations have valid IK solutions.  
Joint limits filter out invalid configurations; the IK returns `false` when no solution passes.

---

## Timing Discovery

**Problem:** First video showed arm only moving ~30° instead of 90°.  
**Cause:** `move_l` is fire-and-forget (no settle detection). Sleep between commands was 2.5s but actual settle time is ~7s.  

**Measured settle time for b=0 ↔ b=90 at Z-home position: ~7 seconds**  
- Multiple joints move simultaneously (J2, J3, J5 all change)
- IK picks a whole-arm reconfiguration, not just wrist rotation
- At 20°/s, 90° of combined joint motion takes ~4-7s to settle

**Fix:** Use 10s between commands for this motion profile.

---

## Confirmed Working move_l Commands

```bash
# Home position, neutral orientation
curl -X POST /robot/move_l -d '{"x":222,"y":0,"z":307,"a":0,"b":0,"c":0}'

# Z-home position, natural orientation  
curl -X POST /robot/move_l -d '{"x":122.58,"y":0,"z":247.74,"a":180,"b":85,"c":180}'

# Z-home position, tip pointing up
curl -X POST /robot/move_l -d '{"x":122.58,"y":0,"z":247.74,"a":180,"b":90,"c":180}'

# Z-home position, alternative wrist config
curl -X POST /robot/move_l -d '{"x":122.58,"y":0,"z":247.74,"a":180,"b":0,"c":180}'
```

---

## Python FK Implementation

Exact match to firmware. Key points:
- Rotation matrix: uses `theta = joint_deg_rad + DH_home_offset`, `alpha` from DH table
- Position: `R0@L1_base + R02@L2_arm + R03@L3_elbow + R06@L6_wrist` (metres × 1000 = mm)
- Orientation: ZYX Euler (a=X, b=Y, c=Z) from R06 rotation matrix
- Link vectors: L1=[D_BS,-L_BS,0], L2=[L_AM,0,0], L3=[-D_EW,0,L_FA], L6=[0,0,L_WT]

---

## Next Steps / Known Gaps

- `move_l` has no settle detection — need sleep or polling after each call
- `lastJoint6D` in firmware is always zero-initialized (not current joints) — affects IK solution selection
- Could add a `GET /robot/pose` endpoint that returns current FK pose (x,y,z,a,b,c) using Python FK
- Could add speed parameter to `move_l` endpoint
