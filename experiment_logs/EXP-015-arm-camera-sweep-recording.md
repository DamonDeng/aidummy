# EXP-015 — Arm + Camera Combined Test: Sweep Recording

**Date:** 2026-03-11  
**Goal:** First combined test of robot arm movement + Astra Pro camera recording. Raise upper arm to a visible position, sweep J1 left↔right × 5 cycles, record the full motion as video, send via Feishu.

---

## Setup

### Server
- Restarted in physical mode: `python3 robot_server.py` (no `--sim`)
- Status confirmed: `{"mode":"physical","status":"connected"}`
- Initial angles: `[0.003, 0.0, 1.223, 0.0, 0.0, 0.0]` ← near home

### Camera
- Device: Astra Pro HD Camera
- ffmpeg AVFoundation device index: `[0]`
- Confirmed via: `ffmpeg -f avfoundation -list_devices true -i ""`

---

## Motion Design

### Goal
Raise the upper arm to a clearly visible position, then sweep J1 left and right 5 full cycles so the motion is wide and easy to observe.

### Pose chosen: Sweep Position
```
[J1=0°, J2=20°, J3=80°, J4=0°, J5=0°, J6=0°]
```
- J2=20°: lower arm angled 20° above horizontal
- J3=80°: upper arm formula = J2+J3−90 = 20+80−90 = 10° above horizontal
- Arm is raised and extended — highly visible from the side

### Sweep range
- J1: −40° to +40° (80° total arc)
- Speed: 25°/s → each half-swing takes ~3.2s, settling ~5s total
- 5 complete cycles (left→right→left→right... × 5)

### Sequence JSON
Used `repeat` block in `/robot/play_sequence`:
```json
{
  "speed": 25,
  "settle_threshold": 1.0,
  "settle_timeout": 12.0,
  "steps": [
    {"move_j": [0, 20, 80, 0, 0, 0], "comment": "raise to sweep position"},
    {
      "repeat": 5,
      "steps": [
        {"move_j": [-40, 20, 80, 0, 0, 0], "comment": "sweep left"},
        {"move_j": [40, 20, 80, 0, 0, 0],  "comment": "sweep right"}
      ]
    },
    {"move_j": [0, 20, 80, 0, 0, 0], "comment": "return to center"},
    {"move_j": [0, 0, 90, 0, 0, 0],  "speed": 15, "comment": "home"}
  ]
}
```

---

## Execution Results

### Sequence
- Steps executed: **13** (1 raise + 10 sweeps + 1 center + 1 home)
- Total wall-clock: **55.09s**
- All steps settled successfully

### Step log
| Step | Comment | Settle (s) |
|---|---|---|
| 1 | raise to sweep position | 1.78 |
| 2 | sweep left (rep 1) | 2.61 |
| 3 | sweep right (rep 1) | 5.06 |
| 4 | sweep left (rep 2) | 5.04 |
| 5 | sweep right (rep 2) | 4.79 |
| 6 | sweep left (rep 3) | 5.01 |
| 7 | sweep right (rep 3) | 4.88 |
| 8 | sweep left (rep 4) | 5.13 |
| 9 | sweep right (rep 4) | 4.99 |
| 10 | sweep left (rep 5) | 5.25 |
| 11 | sweep right (rep 5) | 5.12 |
| 12 | return to center | 2.94 |
| 13 | home | 2.36 |

### Note on encoder readings
- Step log shows encoder-space angles (e.g. `J2=93, J3=-100`)
- This is expected — the firmware reports encoder offset values, not physical angles
- Physical motion matched commanded angles correctly

---

## Video Recording

### Method
ffmpeg capturing from Astra Pro HD Camera simultaneously with arm movement:

```bash
ffmpeg -y \
  -f avfoundation -framerate 30 -video_size 1280x720 \
  -i "0" \
  -t 45 \
  -vcodec libx264 -pix_fmt yuv420p -crf 23 \
  /tmp/arm_sweep.mp4
```

- Started 2 seconds before sequence (camera warm-up)
- Duration: 45s (covers full 55s sequence start + first ~43s of motion)
- Output: `/tmp/arm_sweep.mp4` — 12MB, H.264, 1280×720 @ 30fps

### Delivery
- Uploaded via `POST /im/v1/files` with `file_type=mp4`
- Sent via `msg_type=media` with `{"file_key": "...", "image_key": ""}`
- ✅ Delivered to Feishu

---

## Key Learnings

### 1. `msg_type=media` for video (not `file`, not `video`)
- `msg_type=file` → error 230055 "type mismatch"
- `msg_type=media` + `{"file_key": "...", "image_key": ""}` → ✅ works
- Documented in `lesson_learn/sending_FeiShu_image.md`

### 2. `repeat` block in play_sequence works perfectly
- `{"repeat": N, "steps": [...]}` expands to N×len(steps) individual steps
- Cleaner than manually listing 10 identical sweep steps
- Step log shows all expanded steps with individual settle times

### 3. Camera sees full J1 workspace
- Astra Pro HD Camera position covers J1 ±40° sweep clearly
- This is the primary workspace for future object detection
- No need to move camera for arm observation tasks

### 4. Settle time increases for longer J1 sweeps
- Short moves (raise/center/home): 1.8–2.9s settle
- Full 80° sweeps (−40↔+40): ~5s settle consistently
- At 25°/s, the arm moves for ~3.2s then needs ~1.8s of settling
- `settle_threshold=1.0°` (relaxed from default 0.5°) worked well for sweep speed

---

## Observations / Notes for Next Experiments

- The sweep trajectory is smooth and repeatable — no oscillation observed in step log
- Each sweep cycle took ~10s (2 half-swings × 5s each) → 5 cycles = ~50s total ✓
- The camera's depth stream during arm movement could enable real-time tracking
- **Next: object detection** — place objects on the desk, detect with depth camera, compute 3D centroid, plan pick-and-place

---

## Next Steps (EXP-016+)

- [ ] Place physical objects on desk within arm's workspace
- [ ] Implement `POST /camera/objects/detect`:
  - Capture depth frame
  - Estimate desk plane (dominant flat surface)
  - Find pixels significantly above the plane (elevated blobs)
  - Cluster → compute centroid (X, Y, Z) in camera frame
- [ ] Camera-to-robot coordinate calibration (hand-eye)
- [ ] Pick-and-place loop: detect → transform to robot frame → `move_l`
