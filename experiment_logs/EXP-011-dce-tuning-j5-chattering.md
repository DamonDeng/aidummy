# EXP-011 — DCE Tuning for J5 Chattering During Path Streaming

**Date:** 2026-03-07
**Status:** ⚠️ RESULT NEEDS TO BE VERIFIED (in-office check scheduled Monday)

---

## Problem Statement

During `move_l_path` streaming (b=10→90 wrist sweep), the arm exhibits chattering:
- **Start of sweep** (b=10–30, J5≈88°): smooth, no chattering
- **Middle of sweep** (b≈40–70, J5≈55–20°): visible chattering / vibration
- **End of sweep** (b≈80–90, J5≈7–(−7)°): smooth again

No chattering when arm is stationary.

---

## Root Cause Hypothesis

The DCE controller has **no gravity feedforward**. The only mechanism resisting gravity is the **ki integral** winding up over time. At J5≈45° (mid-sweep), gravitational torque on the wrist is at maximum. During streaming:

1. Gravity pulls J5 → ki integral winds up
2. Next waypoint target arrives (every 80ms) → wound-up integral **overshoots**
3. PID corrects → overshoots other way → chatters at 80ms rhythm
4. At J5≈90° and J5≈0°: gravity torque near zero → no wind-up → smooth

This is a gravity load + integral wind-up interaction, NOT a mechanical or pure PID issue.

---

## DCE Controller (for reference)

Runs at 20 kHz on each joint's STM32:

```
output_mA = (kp × pos_err + integral + kd × vel_err) / 1024
integral  += ki × pos_err + kv × vel_err   (per tick, clamped to ±rated_current×1024)
pos_err   clamped to ±3200 steps (= ±1/16 rev)
```

**Default values (all joints):** kp=200, kv=80, ki=300, kd=250

---

## Infrastructure Added

- **`GET/POST /server/mode`** — switches between `high_level` (default, safe) and `low_level` (exposes tuning endpoints)
- **`GET /robot/joint/{n}/dce`** — read current DCE params (accessible in both modes)
- **`POST /robot/joint/{n}/dce`** — update DCE params (low_level only)
- **`POST /robot/joint/{n}/dce/reset`** — reset to firmware defaults (low_level only)

---

## Experiments Run

Three recordings made, each: z_home → b=10→90 forward sweep → b=90→10 reverse.
All sweeps: 81 waypoints, 80ms step delay via `move_l_path`.

| # | Video file | J5 kp | J5 kv | J5 ki | J5 kd | Change vs default |
|---|---|---|---|---|---|---|
| Baseline | `tune_baseline_s.mp4` | 200 | 80 | **300** | **250** | None (factory defaults) |
| Tuned v1 | `tune_v1_s.mp4`      | 200 | 80 | **150** | **400** | Less wind-up, more damping |
| Tuned v2 | `tune_v2_s.mp4`      | **150** | 80 | **100** | **500** | More aggressive — lower stiffness + heavy damping |

All 3 videos sent to Feishu for comparison.

---

## Current Arm State (as of parking)

J5 is still set to **Tuned v2** params (kp=150, ki=100, kd=500).
Server is in `low_level` mode.

**Before next use, restore defaults:**
```bash
POST /server/mode {"mode": "high_level"}
POST /robot/joint/5/dce/reset
```
Or reboot the J5 motor board to restore firmware defaults.

---

## Expected Outcome (to verify in office)

- Baseline should clearly show chattering in the middle segment
- Tuned v1 (ki=150, kd=400) should reduce chattering noticeably
- Tuned v2 (kp=150, ki=100, kd=500) should be smoothest — or may show signs of being too soft (drooping, slow response)

---

## Next Steps After Verification

1. If v2 is smoothest → use as new J5 baseline for path streaming
2. If v2 is too soft (droops) → find middle ground between v1 and v2
3. Consider adding a `/robot/joint/{n}/dce/save` endpoint that persists tuned values to a JSON file (like sequences.json) so params survive server restart
4. Long-term: investigate gravity compensation feedforward implementation in firmware — would eliminate the need for ki wind-up entirely

---

## Reference

- Firmware DCE source: `2.Firmware/dummy-35motor-fw/Ctrl/Motor/motor.cpp` (line 382)
- Tuning endpoint: `3.Software/robot-server/robot_server.py`
- Related: EXP-010 (path streaming implementation)
