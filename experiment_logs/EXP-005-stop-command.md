# EXP-005 — Stop Command: Emergency Freeze for Running Sequences

**Date:** 2026-03-07  
**Status:** ✅ Complete  
**Participants:** Damon (human), Squilla 🦐 (AI assistant)

---

## Objective

Add a safe stop mechanism to the robot server that:
1. Instantly interrupts any running `play_sequence` mid-execution
2. Freezes the arm at its **current position** (motors stay enabled — arm does NOT fall)
3. Returns clean feedback on where the arm stopped and how many steps completed

---

## Background

Previously, once a sequence started via `POST /robot/play_sequence`, there was no way to stop it short of killing the server process. This was risky for long sequences — if the arm was in an unexpected position mid-way, the only option was to wait it out.

Option 1 (cut motor power via `set_enable(False)`) was rejected immediately — the arm would fall under gravity and risk physical damage. Not acceptable.

---

## Implementation

### New: `POST /robot/stop`

Sets a global `_stop_requested` flag, then reads the current joint angles and re-issues them as the move target — the arm freezes in place with motors engaged.

```json
// Response
{
  "ok": true,
  "frozen_at": [0.0, -30.0, 151.83, 0.0, 0.0, 0.0]
}
```

### New: `_encoder_to_physical()` helper

Inverse of the existing `_physical_to_encoder()`. Required to correctly convert raw encoder angles back to physical angles before passing to `move_j` on real hardware.

```python
def _encoder_to_physical(encoder: list[float]) -> list[float]:
    return [encoder[i] + _REST_POSE[i] for i in range(6)]
```

### Updated: `play_sequence`

- Clears `_stop_requested = False` at the start of every new sequence
- `execute_steps()` checks the flag at the **top of every step** — raises `_StopRequested` if set
- Catches `_StopRequested` and returns a structured response:

```json
{
  "ok": false,
  "stopped": true,
  "steps_completed": 2,
  "frozen_at": [0.0, -30.0, 151.82, 0.0, 0.0, 0.0],
  "total_s": 16.37,
  "step_log": [...]
}
```

### Stop timing behavior

- The stop flag is checked **between steps**, not mid-move
- A step already in progress (e.g. waiting for settle) will complete naturally before the interrupt takes effect
- This is intentional: interrupting a motor mid-command is harder to do safely at the firmware level

---

## Test Results

### Test 1: Stop on idle arm
```
POST /robot/stop → {"ok": true, "frozen_at": [0.0, -30.0, 90.0, 0.0, 0.0, 0.0]}
```
Arm was at rest, responded immediately, held position.

### Test 2: Stop mid-sequence (10-rep elbow up/down loop, speed=20°/s)
- Sequence started: 10 reps × 2 steps = 20 steps total
- Stop fired after ~5 seconds
- Result: `stopped: true`, `steps_completed: 2`, frozen at J3=151.82°
- Arm held its pose, motors stayed on, no drop, no damage

### Video
Recorded and sent via Feishu. Shows:
- Arm cycling elbow up (J3→90°) and down (J3→155°)
- Stop command fires at ~5s
- Arm freezes mid-sequence and holds

---

## API Summary

| Endpoint | Method | Description |
|---|---|---|
| `/robot/stop` | POST | Freeze arm at current position, interrupt any active sequence |

### Normal sequence response
```json
{"ok": true, "steps": 20, "total_s": 42.1, "step_log": [...]}
```

### Stopped sequence response
```json
{"ok": false, "stopped": true, "steps_completed": 2, "frozen_at": [...], "total_s": 16.37, "step_log": [...]}
```

---

## Code Changes

**File:** `3.Software/robot-server/robot_server.py`

| Change | Description |
|---|---|
| `_stop_requested: bool` global | Flag checked by `execute_steps` before each step |
| `_StopRequested` exception class | Internal signal to cleanly unwind the sequence loop |
| `_encoder_to_physical()` helper | Inverse of `_physical_to_encoder`, needed for stop freeze |
| `POST /robot/stop` endpoint | Sets flag + re-issues current angles to freeze arm |
| `play_sequence` updated | Clears flag on entry, catches `_StopRequested`, returns structured stop response |

---

## Lessons Learned

- **Option 1 (motor kill) is dangerous** — never use `set_enable(False)` to stop a moving arm. Gravity wins.
- **Re-issuing current angles = safe freeze** — move_j with the arm's own position is effectively a hold command
- **Between-step interrupt is the right granularity** — firmware-level mid-move interrupts are complex and risky; step boundaries are clean and safe
- **`_read_angles()` returns encoder space on real hardware** — always convert back to physical before calling `move_j`

---

## Next Ideas

- "Pause and resume" — freeze with ability to continue the remaining steps later
- "Slow stop" — gradually decelerate to current position over N seconds instead of instant hold
- Interrupt `_wait_for_settle` too (for very long settle timeouts), not just between steps
