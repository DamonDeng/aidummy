# EXP-019: Yellow Cover Swap — Hardware Upgrade & First Look

**Date:** 2026-03-12 (afternoon)
**Status:** ✅ HARDWARE UPGRADE COMPLETE / 🔬 VISION CHARACTERIZATION IN PROGRESS
**Objective:** Swap the robot arm's 3D-printed link covers from the original color to new yellow ones, then characterize how the new color affects depth camera quality and color segmentation.

---

## Background

The arm's original 3D-printed covers had been a known limitation for the computer vision work. Darker or saturated colors cause inconsistent HSV segmentation, and the Orbbec Astra Pro structured-light depth camera performs poorly on surfaces that don't reflect IR well. Damon designed and printed a new set of covers in yellow — a color chosen deliberately for its IR reflectance and easy HSV isolation.

---

## The Swap

The arm was parked to the rest/folded pose `[J1=0, J2=-73, J3=180, J4=0, J5=0, J6=0]` before powering off, then the covers were physically swapped on the bench.

### The "PA!" Incident

During the installation, Damon heard an inauspicious snap — a sharp "PA!" sound from somewhere inside the arm. The immediate concern was obvious: had one of the driver boards or a motor controller cracked under pressure?

The arm was powered back on and reconnected. First signs were not encouraging — ALL six joints showed zero response during the initial test. For a moment, it really did look like something fundamental had broken.

Then the obvious was remembered: after any power cycle, the motors default to disabled. One `POST /robot/enable {"enable": true}` later, J1 swung to 15° exactly as commanded.

The full 6-joint diagnostic was run: J1 through J6, each tested with a ±20–40° movement, all reporting exact commanded angles. Every single joint: ✅.

**The "PA!" was a plastic clip snapping into place on the new cover. Nothing more.**

In retrospect, the incident was useful — it surfaced the gap in our pre-test procedure. The motor enable step is now explicitly documented as required after any power cycle, and the joint diagnostic script exists and is ready to run whenever there's doubt.

---

## Results: Depth Camera

**Before (old covers):** Significant IR dropout on darker surfaces, arm partially invisible in depth frame.

**After (yellow covers):** 74% valid pixels across the 1280×1024 depth frame.

The arm is now largely visible in depth, with one exception: a small black hole around J5. This is a geometry issue — the J5 joint area has an open frame structure where the IR pattern passes through rather than reflecting back. This is structural, not color-related, and is unlikely to change without a hardware redesign.

For future 3D arm modeling, the depth data now gives us the major link geometry. The J5 gap will require interpolation or FK-based infill.

---

## Results: Color Segmentation

The test image revealed an important permanent constraint:

**The arm is — and will always be — three colors:**
- 🔴 **Red**: CNC-machined structural parts (joint housings, motor covers). These are the load-bearing aluminum components. They cannot be repainted or replaced without redesigning the arm.
- 🟡 **Yellow**: 3D-printed link covers. These are cosmetic/protective shells. They can be reprinted in any color.
- ⚫ **Black**: base, cable management.

A single-color segmentation approach (yellow only) was found to produce 11 fragmented blobs with a largest region of only ~20k pixels — because the red CNC joints break continuity between yellow cover segments.

**The revised strategy:** detect red OR yellow as "arm":
```python
mask_red    = cv2.inRange(hsv, [0,100,60], [12,255,255]) |
              cv2.inRange(hsv, [158,100,60], [180,255,255])  # red wraps around
mask_yellow = cv2.inRange(hsv, [18,80,80], [38,255,255])
mask_arm    = mask_red | mask_yellow
```

This approach still has false positives from scene objects (cardboard boxes hit the tan/yellow range; chair fabric hits orange-red). Background subtraction therefore remains the most reliable method for clean arm isolation — it is color-agnostic and captures exactly what moved, regardless of color complexity.

**Open question:** The J6 end-effector flange — is it a CNC red part or a printed yellow part? If it is yellow, its position at the extreme distal end of the arm gives it unique potential: it would be the only yellow region consistently at the arm tip, which could enable tip localization via "find the distal-most yellow blob." This is worth testing.

---

## What This Experiment Changes for the Vision System

| Aspect | Before | After |
|---|---|---|
| Depth arm coverage | Partial (IR dropout on dark covers) | ~74% (much better) |
| Color segmentation | Red only → fragmented | Red + Yellow → more complete |
| False positive risk | Low (red arm in non-red scene) | Higher (yellow/tan objects in scene) |
| Background subtraction | Still best | Still best |
| 3D arm modeling potential | Limited depth data | Viable with depth + FK infill |

---

## Next Steps

- [ ] Test combined red+yellow color segmentation for arm mask quality
- [ ] Rebuild background model with yellow arm appearance
- [ ] Re-run whole-arm dataset generator (`dataset_generator.py`) with new covers for updated training data
- [ ] Determine J6 flange color — test whether distal yellow blob = reliable tip indicator
- [ ] Explore depth + RGB fusion for 3D arm reconstruction at multiple poses

---

## Notes for the Future

Reading this years later: the "PA!" moment was genuinely alarming at the time. Six joints showing zero response with a mystery snap sound — that's the kind of moment you remember. It turned out to be a plastic clip. The lesson isn't "don't panic" — it's "always check the simple things first." The motor enable step was sitting right there, obvious in hindsight, easy to miss in the moment when you're expecting broken hardware.

The yellow covers look good. The arm is starting to look like something designed, not just assembled.
