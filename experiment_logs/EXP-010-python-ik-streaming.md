# EXP-010 — Python IK Solver + Cartesian Path Streaming

**Date:** 2026-03-07  
**Goal:** Implement a Python-side IK solver to enable smooth Cartesian path streaming via batched MoveJ commands

---

## Motivation

Previous `move_l` approach:
- Fire-and-forget: IK computed on STM32, one call at a time
- No settle detection, must wait ~7-10s between calls
- 81 waypoints × 10s = ~13 minutes for a b=10→90 sweep

Target: pre-compute IK in Python, stream MoveJ commands rapidly → PID "chases" the path → smooth, continuous motion in seconds.

---

## Key Insight: MoveL is just IK + MoveJ

From firmware `dummy_robot.cpp`:
```cpp
bool DummyRobot::MoveL(...) {
    SolveIK(pose6D, lastJoint6D, ikSolves);  // analytical IK
    // filter by joint limits, pick solution closest to currentJoints
    return MoveJ(best_solution);             // joint-space move
}
```

The STM32 `currentJoints` (real state) is used for solution selection — NOT zeros. This is important: chaining IK solutions properly requires passing the previous solution as the reference for the next solve.

---

## Python IK Implementation

**File:** `3.Software/robot-server/ik_solver.py`

Exact port of firmware `DOF6Kinematic::SolveIK` — analytical closed-form 6-DOF IK:

1. **Euler → rotation matrix** (`_euler_to_rot`): firmware convention R = Rz(c) @ Ry(b) @ Rx(a)
   - Note: firmware variable naming is confusing (`cc/sc` = cos/sin of `a`, `ca/sa` = cos/sin of `c`) — ported exactly
2. **Wrist centre**: `P_w = P_target - R06 @ L6_wrist`
3. **J1**: `atan2(P_w[1], P_w[0])` — two solutions (shoulder front/back)
4. **J2, J3**: law-of-cosines in the shoulder plane — two solutions each (elbow up/down)
5. **J4, J5, J6**: wrist decomposition from `R36 = R30ᵀ @ R06` — two solutions (wrist flip)
6. **8 total candidate solutions** → filter by joint limits → pick closest to `current_joints`

**Joint limits used:**
| Joint | Min (°) | Max (°) |
|---|---|---|
| J1 | -45 | 45 |
| J2 | -73 | 90 |
| J3 | 35 | 180 |
| J4 | -180 | 180 |
| J5 | -120 | 120 |
| J6 | -720 | 720 |

---

## Performance

```
Speed: 1000 IK solves in 49.6ms → 0.050ms per solve (20,156 solves/sec)
Batch: 81 poses (b=10→90 sweep) solved in 3.6ms total
```

**Round-trip accuracy (FK → IK):**
- HOME  [0,0,90,0,0,0]    → IK error = 0.000000° ✅
- Z-HOME [0,-45,140,0,0,0] → IK error = 0.000000° ✅
- REST  [0,-73,180,0,0,0]  → boundary case (J3=180 at limit, minor numerical issue)

---

## New API Endpoint: POST /robot/move_l_path

**File:** `3.Software/robot-server/robot_server.py`

```json
{
  "poses": [[x, y, z, a, b, c], ...],
  "step_delay_ms": 80,
  "tcp_apply": true
}
```

**How it works:**
1. Read current joint angles from arm (for IK seed)
2. Optionally apply TCP offset to each waypoint
3. Batch-solve IK for all waypoints (chained: each solution seeds the next)
4. Stream MoveJ commands at `step_delay_ms` intervals
5. PID controller "chases" the moving target → smooth continuous motion

**Response:** `{"ok": true, "steps_sent": 81, "steps_failed": 0}`

---

## Demo Results

Sweep b=10°→90° at Z-home tip position (122.58, 0, 247.74mm):

| Method | Total time | Motion quality |
|---|---|---|
| 81 × `move_l`, 10s settle | ~13 minutes | Jerky, stop-start |
| 81 × `move_l_path`, 80ms delay | **6.5 seconds** | **Smooth, fluid** |
| IK pre-computation | 3.6ms | — |

Video: `stream_sweep.mp4` — forward sweep (b=10→90) then reverse (b=90→10), 80ms steps.

---

## Key Learnings

1. **Streaming MoveJ = smooth Cartesian paths** — no need for settle waits; PID transitions continuously between targets
2. **IK chaining is critical** — each solve must use the previous solution as reference to avoid elbow/shoulder "flips" mid-path
3. **Python IK is fast enough** — 20K solves/sec means real-time path generation for even complex trajectories
4. **step_delay_ms tuning**: lower = faster/smoother but arm may lag; 80ms is a good default; tune down for faster moves

---

## Next Steps / Ideas

- Add `move_l_circle` endpoint: generate circular arc waypoints automatically
- Add `move_l_linear` endpoint: interpolate linearly in Cartesian space between two points
- Tune `step_delay_ms` based on arm speed and path curvature
- Handle IK failures mid-path (currently skipped with a counter)
- Expose IK batch API directly for external path planners
