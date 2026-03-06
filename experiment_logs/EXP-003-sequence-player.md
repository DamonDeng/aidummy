# EXP-003 — Sequence Player: Natural Language to Motion + Video

**Date:** 2026-03-06  
**Status:** ✅ Complete  
**Participants:** Damon (human), Squilla 🦐 (AI assistant)

---

## Objective

Build a motion sequence system that:
1. Accepts a sequence of joint moves via a new HTTP API endpoint
2. Executes steps with smart settling detection (not fixed timers)
3. Records the motion as video and sends it via Feishu
4. Allows Squilla to translate natural language descriptions into sequences

---

## Background

Prior to this experiment, arm control was one command at a time via `/robot/move_j`. 
The goal was to enable choreographed multi-step motions described in plain text.

---

## Design Decisions

### Timing Strategy: Smart Settling (A-2)
Two options were considered:
- **A-1 (fixed wait):** simple but unreliable — too short/long depending on move distance
- **A-2 (smart polling):** poll joint angles at ~6 Hz until all joints within threshold for 3 consecutive readings, with fallback timeout

**[KEY RESULT]** Chose A-2. Measured ~3s actual settling per step vs. 10s fixed timeout — 3× more efficient.

### Sequence Format
```json
{
  "speed": 25,
  "settle_threshold": 0.5,
  "settle_timeout": 12,
  "steps": [
    {"move_j": [0, -30, 110, 0, 0, 0], "hold": 0.0, "comment": "cycle 1 up"},
    {"move_j": [0, -30, 155, 0, 0, 0], "comment": "cycle 1 down"}
  ]
}
```

Each step supports:
- `move_j` — 6 joint angles in degrees (physical angles, same as firmware)
- `move_l` — Cartesian [x, y, z, a, b, c] (IK handled by firmware)
- `speed` — per-step override (deg/s)
- `hold` — extra seconds to pause after settling
- `comment` — human-readable label, ignored by executor

---

## Implementation

### New API Endpoint: `POST /robot/play_sequence`

Added to `3.Software/robot-server/robot_server.py`.

**Key implementation detail — encoder vs. physical angle mismatch:**

`move_j()` takes physical angles (REST = {0, -73, 180, 0, 0, 0}), but `joint_X.angle` returns motor encoder values (0 = REST). The settling check must compare in the same space.

**[LESSON]** Initial implementation compared physical targets directly to encoder readings — always hit timeout. Fix: convert physical target → encoder space before settling check.

```python
_REST_POSE = [0.0, -73.0, 180.0, 0.0, 0.0, 0.0]

def _physical_to_encoder(physical):
    return [physical[i] - _REST_POSE[i] for i in range(6)]
```

Settling logic: poll at 6 Hz, require 3 consecutive readings all within `settle_threshold` degrees. Falls back to `settle_timeout` if arm stalls.

**Response:**
```json
{
  "ok": true,
  "steps": 11,
  "total_s": 34.15,
  "step_log": [
    {"step": 0, "comment": "start position", "angles": [...], "settle_s": 3.04, "total_s": 3.05},
    ...
  ]
}
```

---

## Safety Rules Established

**[KEY RESULT]** J1 (base rotation) permanently forbidden — arm is too close to the MacBook. All sequences must keep J1=0. Documented in `learning_docs/for_agents_using_this_robot_arm.md`.

---

## First Sequence: Upper Arm Up/Down × 5

**Natural language description:** "Move upper arm (J3) up and down 5 times"

**Translated sequence:** J3 oscillates between 110° (up) and 155° (down), J1=0 locked, 11 steps total (1 start + 10 cycles), speed=25°/s.

**Execution results:**

| Metric | Value |
|--------|-------|
| Steps | 11 |
| Total duration | 34.15s |
| Avg settle time per step | ~3.1s |
| All steps completed | ✅ |

**Video:** [`arm_updown_5cycles.mp4`](assets/EXP-003/arm_updown_5cycles.mp4)  
*Upper arm (J3) oscillating between 110° and 155°, 5 complete cycles*

---

## Video Sending — New Lesson Learned

**[LESSON]** Feishu video sending requires a specific two-step method:
1. Upload with `file_type=mp4` to `/im/v1/files` → get `file_key`
2. Send with `msg_type=media` and `content: {"file_key": "...", "image_key": ""}` — the `image_key` field must be present but can be empty

Wrong approaches that fail:
- `msg_type=file` → "type does not match"
- `file_type=video` upload → upload fails
- `msg_type=video` → invalid

Documented in `lesson_learn/sending_FeiShu_image.md`.

---

## Working Mode System

Two modes established for arm control sessions:

| Mode | Trigger | Behavior |
|------|---------|---------|
| 🏗️ Building | Default / "building mode" | Detailed, explanatory, collaborative |
| 🏃 Running | Chinese message starting with 螳螂虾 | Fast execute, no explanation, action + photo |

Running mode stays active until explicitly told "building mode".

---

## Key Results Summary

**[KEY RESULT]** `/robot/play_sequence` endpoint implemented and working ✅  
**[KEY RESULT]** Smart settling: ~3s per step vs. 10s fixed timeout (3× faster) ✅  
**[KEY RESULT]** Full pipeline: text → sequence → execution → video → Feishu ✅  
**[KEY RESULT]** Video sending method for Feishu documented and working ✅  
**[KEY RESULT]** J1 safety lock established and documented ✅  

---

## Open Items

- [ ] Add `repeat` count to sequence steps (avoid writing 10 identical steps for 5 cycles)
- [ ] Add `move_l` settling support (currently uses fixed wait, no angle target for IK)
- [ ] Try Cartesian (`move_l`) sequences
- [ ] Try longer choreography (wave, nod, etc.)
- [ ] Explore faster speed settings and their effect on settling time
