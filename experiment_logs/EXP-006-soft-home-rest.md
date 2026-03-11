# EXP-006 — Soft Home & Soft Rest: Settle-Confirmed Pose Commands

**Date:** 2026-03-07  
**Status:** ✅ Complete  
**Participants:** Damon (human), Squilla 🦐 (AI assistant)

---

## Objective

Replace the buggy firmware `homing()` and `resting()` commands with reliable
"soft" versions that:
1. Use `move_j` (proven reliable) instead of firmware-level pose commands
2. Confirm arrival via `_wait_for_settle` — not fire-and-forget
3. Return the actual settled angles and settle time
4. Respect `/robot/stop` — exit the settle loop early if stop is requested
5. Auto-clear the stop flag when called (explicit command = intentional move)

---

## Background: Why the Firmware Versions Were Broken

The original endpoints were:

```python
@app.post("/robot/homing")
def homing():
    get_robot().robot.homing()  # firmware command
    return {"ok": True}         # returns immediately — no confirmation
```

The firmware `homing()` / `resting()` commands have a bug: they cannot reliably
detect when the arm has finished moving. The API had no choice but to return
instantly and leave the caller guessing. In practice this required adding
arbitrary `sleep()` delays on the client side, which was fragile and slow.

---

## Implementation

### Replaced endpoints

Both `POST /robot/homing` and `POST /robot/resting` now use the same
move_j + settle pattern proven in `play_sequence`:

```
explicit call → clear stop flag → enable motors → set speed
             → move_j(target_pose) → _wait_for_settle(check_stop=True)
             → return confirmed angles + settle_s
```

### Target poses

| Endpoint | Physical pose | Description |
|---|---|---|
| `/robot/homing` | `[0, 0, 90, 0, 0, 0]` | Upright ready position |
| `/robot/resting` | `[0, -73, 180, 0, 0, 0]` | Folded storage position |

### Request body: `SoftMoveBody`

```json
{
  "speed": 15.0,            // deg/s — default gentle, override freely
  "settle_threshold": 0.5,  // degrees — "close enough"
  "settle_timeout": 20.0    // max wait seconds
}
```
All fields optional — defaults are safe out of the box.

### Normal response

```json
{
  "ok": true,
  "pose": "home",
  "target": [0.0, 0.0, 90.0, 0.0, 0.0, 0.0],
  "angles": [0.0, 0.0, 90.0, 0.0, 0.0, -0.0],
  "settle_s": 8.79
}
```

### Stopped response (if `/robot/stop` fires during move)

```json
{
  "ok": false,
  "stopped": true,
  "frozen_at": [0.0, -58.24, 161.77, 0.0, 0.0, 0.0],
  "settle_s": 3.27
}
```

### Stop flag behaviour

| Command | Stop flag effect |
|---|---|
| `/robot/homing` | **Clears** flag on entry (intentional move) |
| `/robot/resting` | **Clears** flag on entry (intentional move) |
| `/robot/play_sequence` | **Clears** flag on entry |
| `/robot/stop` | **Sets** flag |

This means after a stop, calling `homing` or `resting` directly works —
no separate "clear stop" step needed.

### Also updated: `_wait_for_settle`

Added optional `check_stop: bool = False` parameter. When `True`, the poll
loop exits early as soon as `_stop_requested` is set — typically within one
poll cycle (~150ms).

```python
def _wait_for_settle(r, target, threshold, timeout, check_stop=False):
    ...
    if check_stop and _stop_requested:
        break
    ...
```

### New request model: `SoftMoveBody`

Added alongside existing models in the request models section.

---

## Test Results

### Test 1: soft_rest from offset position
- Start: `[20, -20, 110, 0, 30, 0]` (manual offset)
- Result: `[0, -73, 180, 0, 0, 0]` — confirmed ✅
- Settle time: **7.31s**

### Test 2: soft_home from rest
- Start: `[0, -73, 180, 0, 0, 0]`
- Result: `[0, 0, 90, 0, 0, 0]` — confirmed ✅
- Settle time: **8.79s**

### Test 3: stop mid-homing (speed=8°/s)
- Stop fired at 3s
- Arm froze at `[0, -58.24, 161.77, 0, 0, 0]`
- `homing` returned `stopped: true`, `settle_s: 3.27` ✅
- Both `/robot/stop` and `/robot/homing` reported the same `frozen_at` ✅

### Videos
Both recorded and sent to Damon via Feishu:
- `soft_rest_then_home.mp4` — full rest → home cycle with confirmed settle
- `stop_mid_homing.mp4` — stop fires mid-travel, arm freezes cleanly

---

## Code Changes

**File:** `3.Software/robot-server/robot_server.py`

| Change | Description |
|---|---|
| `SoftMoveBody` model | New request body with `speed`, `settle_threshold`, `settle_timeout` |
| `_wait_for_settle` | Added `check_stop=False` param — exits loop early when stop requested |
| `POST /robot/homing` | Replaced firmware call with `move_j` + settle, stop-aware, clears flag |
| `POST /robot/resting` | Replaced firmware call with `move_j` + settle, stop-aware, clears flag |

---

## Lessons Learned

- **Fire-and-forget firmware commands are unreliable** — always confirm with
  settle detection when the caller needs to know the arm arrived
- **Explicit commands should clear the stop flag** — if the user calls
  `homing`, they clearly want to move. Don't require a separate reset step.
- **`check_stop` in the wait loop** is the right pattern for interruptible
  blocking waits — one parameter, backward compatible, ~150ms response time

---

## Next Ideas

- `POST /robot/move_j_confirmed` — a confirmed version of raw `move_j` using
  the same settle detection (currently `move_j` returns immediately)
- Configurable poses — allow user to save/name custom poses and soft-go to them
