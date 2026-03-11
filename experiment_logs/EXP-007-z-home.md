# EXP-007 — Z-Home: Natural Z-Shape Resting Pose

**Date:** 2026-03-07  
**Status:** ✅ Complete  
**Participants:** Damon (human), Squilla 🦐 (AI assistant)

---

## Objective

Add a `POST /robot/z_home` endpoint that moves the arm into a natural Z-shaped
resting pose — more visually appealing and ergonomic than the standard home
position `[0, 0, 90, 0, 0, 0]`.

---

## Background

The existing `homing` pose `[J2=0, J3=90]` places the lower arm vertical and
upper arm horizontal — functional but stiff-looking. The goal was a pose where:
- Lower arm tilts back at a natural angle
- Upper arm is approximately horizontal
- The overall shape looks like a **Z** from the side

---

## Key Lesson: Joint Mapping Correction

During this experiment a wrong assumption was identified and corrected:

| | Wrong (initial) | Correct |
|---|---|---|
| J2 controls | upper arm | **lower arm** |
| J3 controls | lower arm | **upper arm** |

This matters for angle calculations. The correct formula for upper arm angle
from horizontal is:

```
upper_arm_angle = J2 + J3 − 90°
```

**Verification:**
- HOME `[J2=0, J3=90]` → 0 + 90 − 90 = **0°** (horizontal) ✓
- REST `[J2=−73, J3=180]` → −73 + 180 − 90 = **17°** (parallel to lower arm) ✓

**For horizontal upper arm:** J3 = 90 − J2

---

## Pose Exploration

Multiple angles were tested via `move_j` before committing to the server:

| Attempt | J2 | J3 | Upper arm angle | Result |
|---|---|---|---|---|
| 1 (wrong formula) | 0 | 60 | +60° below horizontal | ❌ Wrong — J2/J3 mapping misunderstood |
| 2 | −45 | 45 | −45+45−90 = −90° (vertical) | ❌ Totally wrong |
| 3 | −45 | 135 | −45+135−90 = 0° (horizontal) | ✅ Correct shape but slightly low |
| 4 ✅ | −45 | 140 | −45+140−90 = **+5°** | ✅ Approved by Damon |

Final pose: **`[0, −45, 140, 0, 0, 0]`**

---

## Implementation

### Final Z-Home Pose

| Joint | Angle | Description |
|---|---|---|
| J1 | 0° | Base centered |
| J2 | −45° | Lower arm tilted back 45° from vertical |
| J3 | 140° | Upper arm 5° above horizontal — natural, not stiff |
| J4 | 0° | Wrist neutral |
| J5 | 0° | Wrist neutral |
| J6 | 0° | Wrist neutral |

### New constant

```python
_Z_HOME_POSE = [0.0, -45.0, 140.0, 0.0, 0.0, 0.0]
```

### New endpoint: `POST /robot/z_home`

Same pattern as `soft_home` / `soft_rest`:

```json
// Response
{
  "ok": true,
  "pose": "z_home",
  "target": [0.0, -45.0, 140.0, 0.0, 0.0, 0.0],
  "angles": [0.0, -45.0, 140.0, 0.0, 0.0, 0.0],
  "settle_s": 4.5
}
```

- Clears `_stop_requested` on entry
- Enables motors, sets speed
- Calls `move_j` then `_wait_for_settle(check_stop=True)`
- Returns confirmed angles + settle time
- Respects `/robot/stop`

---

## Test Results

| Test | Result |
|---|---|
| `POST /robot/z_home` from rest | Settled at `[0, −45, 140, 0, 0, 0]` in **4.5s** ✅ |
| Visual confirmation (photo sent) | Z shape approved by Damon ✅ |

---

## Code Changes

**File:** `3.Software/robot-server/robot_server.py`

| Change | Description |
|---|---|
| `_Z_HOME_POSE` constant | `[0, −45, 140, 0, 0, 0]` added alongside HOME and REST |
| `POST /robot/z_home` endpoint | Full soft-move with settle detection and stop support |

---

## Lessons Learned

- **J2 = lower arm, J3 = upper arm** — opposite of initial assumption. Always verify joint mapping on real hardware before building.
- **Upper arm angle formula: J2 + J3 − 90°** — derived from two known poses (HOME and REST), verified against real arm.
- **Horizontal upper arm constraint:** J3 = 90 − J2. With J3_min = 35°, the most extreme tilt is J2 = −55°.
- **Test with `move_j` first** — one photo from the real arm is worth more than any calculation. Iterate visually, then bake into server code.

---

## All Named Poses (as of EXP-007)

| Pose | J1 | J2 | J3 | J4 | J5 | J6 | Upper arm from horizontal |
|------|----|----|----|----|----|----|---|
| Home | 0 | 0 | 90 | 0 | 0 | 0 | 0° (exactly horizontal) |
| Rest | 0 | −73 | 180 | 0 | 0 | 0 | 17° (folded storage) |
| Z-Home | 0 | −45 | 140 | 0 | 0 | 0 | +5° (natural Z shape) |
