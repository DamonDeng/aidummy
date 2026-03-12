"""
hand_dataset_generator.py — Auto-label ONLY the hand/wrist part of the robot arm.

Strategy:
  For each base arm pose (J1/J2/J3):
    1. Capture reference image at (J4=0, J5=0, J6=0)
    2. Vary only J4/J5/J6 (wrist joints)
    3. mask_changed = XOR(reference_mask, current_mask)
       → only the hand/wrist moved → this IS the hand region
    4. Bounding box around changed region = hand label

Output: YOLO dataset with class 0 = "robot_hand"
"""

import cv2
import numpy as np
import json
import time
import requests
import subprocess
from pathlib import Path
from datetime import datetime

SERVER  = "http://127.0.0.1:3001"
CAMERA  = "Astra Pro HD Camera"
LIMITS  = dict(j1=(-45,45), j2=(-73,90), j3=(35,180), j4=(-180,180), j5=(-120,120), j6=(-720,720))

# ── Base arm poses: J1/J2/J3 fixed, wrist at zero ────────────────────────────
BASE_POSES = [
    ( 0, -30, 140),   # forward, raised
    ( 0, -20, 120),   # forward, mid
    ( 0,   0,  90),   # home
    (25, -30, 140),   # rotated right, raised
    (-25, -30, 140),  # rotated left, raised
    (25, -20, 120),   # rotated right, mid
    (-25, -20, 120),  # rotated left, mid
    ( 0, -40, 150),   # more raised
    (35, -40, 150),   # far right raised
    (-35, -40, 150),  # far left raised
]

# ── Wrist sweep: J4/J5/J6 variations ─────────────────────────────────────────
WRIST_POSES = []
for j4 in (-90, -45, 0, 45, 90):
    for j5 in (-90, -45, 0, 45, 90):
        for j6 in (-180, -90, 0, 90, 180):
            WRIST_POSES.append((j4, j5, j6))
# Filter: skip (0,0,0) — that's the reference pose, not a sample
WRIST_POSES = [(j4,j5,j6) for j4,j5,j6 in WRIST_POSES if not (j4==0 and j5==0 and j6==0)]


def safe(j1,j2,j3,j4=0,j5=0,j6=0):
    for v,(lo,hi) in zip([j1,j2,j3,j4,j5,j6], LIMITS.values()):
        if not (lo <= v <= hi): return False
    if j2 + j3 - 90 < -20: return False
    return True

def move_j(j1,j2,j3,j4=0,j5=0,j6=0,speed=25):
    r = requests.post(f"{SERVER}/robot/move_j",
                      json=dict(j1=j1,j2=j2,j3=j3,j4=j4,j5=j5,j6=j6,speed=speed))
    return r.json().get("ok", False)

def capture(path, warmup=1.2):
    subprocess.run(
        ["/opt/homebrew/bin/imagesnap", "-d", CAMERA, "-w", str(warmup), str(path)],
        capture_output=True
    )
    return Path(path).exists()

def arm_mask(img, background, threshold=28):
    diff = np.abs(img.astype(np.float32) - background.astype(np.float32)).max(axis=2)
    m = (diff > threshold).astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9,9))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, iterations=3)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  k, iterations=1)
    return m

def hand_mask_from_tip(mask_arm, tip_fraction=0.30):
    """
    Extract the hand region from the arm mask.

    The hand is always at the DISTAL end of the arm.
    Strategy:
      1. Find arm base (centroid of bottom 10% of arm pixels)
      2. For each arm pixel, compute distance along arm axis from base
      3. Keep the top `tip_fraction` of pixels by distance = hand region
    """
    ys, xs = np.where(mask_arm > 0)
    if len(ys) < 200:
        return None

    # Base = centroid of the lowest 15% of arm pixels (largest y = bottom of image)
    thresh_y = np.percentile(ys, 85)
    base_mask = ys >= thresh_y
    base_x = float(xs[base_mask].mean())
    base_y = float(ys[base_mask].mean())

    # Distance of each arm pixel from base
    dists = np.sqrt((xs - base_x)**2 + (ys - base_y)**2)

    # Keep top tip_fraction by distance
    cutoff = np.percentile(dists, (1.0 - tip_fraction) * 100)
    hand_ys = ys[dists >= cutoff]
    hand_xs = xs[dists >= cutoff]

    if len(hand_ys) < 100:
        return None

    hand_mask = np.zeros_like(mask_arm)
    hand_mask[hand_ys, hand_xs] = 255

    # Clean up
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11,11))
    hand_mask = cv2.morphologyEx(hand_mask, cv2.MORPH_CLOSE, k, iterations=2)
    return hand_mask

def bbox_yolo(mask, H, W):
    x,y,w,h = cv2.boundingRect(mask)
    if w == 0 or h == 0: return None
    # Add 10% padding
    pad_x, pad_y = int(w*0.1), int(h*0.1)
    x = max(0, x-pad_x); y = max(0, y-pad_y)
    w = min(W-x, w+2*pad_x); h = min(H-y, h+2*pad_y)
    return (x+w/2)/W, (y+h/2)/H, w/W, h/H


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--output-dir", default="/tmp/hand_dataset")
    ap.add_argument("--bg-model",   default="/tmp/bg_model.npy")
    ap.add_argument("--max-per-base", type=int, default=None,
                    help="Limit wrist poses per base (default: all)")
    args = ap.parse_args()

    # Check server
    status = requests.get(f"{SERVER}/robot/status").json()
    print(f"Robot: {status}")
    assert status["status"] == "connected", "Robot not connected"

    # Load or build background
    bg_path = Path(args.bg_model)
    if bg_path.exists():
        bg = np.load(str(bg_path))
        print(f"Loaded background: {bg.shape}")
    else:
        print(f"Background not found — building fresh (arm will sweep)...")
        from dataset_generator import build_background
        bg = build_background(str(bg_path))
        if bg is None:
            print("ERROR: background build failed"); exit(1)
    H, W = bg.shape[:2]

    # Output dirs
    out = Path(args.output_dir)
    for d in ["images","labels","masks","metadata"]:
        (out/d).mkdir(parents=True, exist_ok=True)

    wrist_poses = WRIST_POSES
    if args.max_per_base:
        # Sample evenly across the wrist range
        step = max(1, len(wrist_poses) // args.max_per_base)
        wrist_poses = wrist_poses[::step][:args.max_per_base]

    total_planned = len(BASE_POSES) * len(wrist_poses)
    print(f"\nBase poses:   {len(BASE_POSES)}")
    print(f"Wrist poses:  {len(wrist_poses)} per base")
    print(f"Total samples: {total_planned}")
    print(f"Est. time: ~{total_planned*4//60}m\n")

    captured = 0; skipped = 0; sample_idx = 0
    start = time.time()

    for base_idx, (j1,j2,j3) in enumerate(BASE_POSES):
        if not safe(j1,j2,j3):
            print(f"Base {base_idx} unsafe, skip"); continue

        print(f"\n{'─'*55}")
        print(f"Base {base_idx+1}/{len(BASE_POSES)}: J1={j1:+d} J2={j2:+d} J3={j3:+d}")
        print(f"{'─'*55}")

        # ── Reference capture at this base pose (wrist=0) ──
        move_j(j1,j2,j3, speed=20); time.sleep(4)
        ref_path = out/f"_ref_base{base_idx:02d}.jpg"
        if not capture(ref_path):
            print("  Reference capture failed, skipping base"); continue
        ref_img = cv2.imread(str(ref_path))
        mask_ref = arm_mask(ref_img, bg)
        ref_pixels = int(np.sum(mask_ref > 0))
        print(f"  Reference captured — arm pixels: {ref_pixels:,}")
        for w_idx, (j4,j5,j6) in enumerate(wrist_poses):
            if not safe(j1,j2,j3,j4,j5,j6): continue

            stem = f"hand_{sample_idx:04d}_b{base_idx:02d}_j{j1:+d}_{j2:+d}_{j3:+d}__w{j4:+d}_{j5:+d}_{j6:+d}"
            img_path  = out/"images"/f"{stem}.jpg"
            lbl_path  = out/"labels"/f"{stem}.txt"
            mask_path = out/"masks"/f"{stem}.png"
            meta_path = out/"metadata"/f"{stem}.json"

            if img_path.exists():
                sample_idx += 1; captured += 1; continue

            elapsed = time.time()-start
            eta = (elapsed/(sample_idx+1))*(total_planned-sample_idx-1) if sample_idx>0 else 0
            print(f"  [{sample_idx+1:4d}/{total_planned}] "
                  f"J4={j4:+4d} J5={j5:+4d} J6={j6:+4d}  ETA:{eta/60:.1f}min", end="  ", flush=True)

            move_j(j1,j2,j3,j4,j5,j6, speed=30); time.sleep(3.2)

            if not capture(img_path):
                print("FAIL"); skipped += 1; sample_idx += 1; continue

            cur_img = cv2.imread(str(img_path))
            mask_cur = arm_mask(cur_img, bg)
            hand = hand_mask_from_tip(mask_cur, tip_fraction=0.28)

            if hand is None:
                print("NO HAND REGION"); skipped += 1; sample_idx += 1; continue

            bbox = bbox_yolo(hand, H, W)
            if bbox is None:
                print("NO BBOX"); skipped += 1; sample_idx += 1; continue

            cx,cy,bw,bh = bbox
            hand_pixels = int(np.sum(hand > 0))

            # YOLO label: class 1 = robot_hand
            lbl_path.write_text(f"1 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")
            cv2.imwrite(str(mask_path), hand)
            meta_path.write_text(json.dumps({
                "idx": sample_idx, "base_idx": base_idx,
                "base_joints": {"j1":j1,"j2":j2,"j3":j3},
                "wrist_joints": {"j4":j4,"j5":j5,"j6":j6},
                "bbox_yolo": list(bbox),
                "hand_pixels": hand_pixels,
                "ref_arm_pixels": ref_pixels,
                "timestamp": datetime.now().isoformat()
            }, indent=2))

            print(f"✓  hand_px={hand_pixels:,}  bbox=({cx:.2f},{cy:.2f},{bw:.2f},{bh:.2f})")
            captured += 1; sample_idx += 1

        # Return wrist to zero before next base
        move_j(j1,j2,j3, speed=25); time.sleep(2)

    # Park
    move_j(0,-73,180, speed=15)

    duration = int(time.time()-start)
    summary = {"total_planned": total_planned, "captured": captured,
               "skipped": skipped, "duration_sec": duration}
    (out/"dataset_summary.json").write_text(json.dumps(summary, indent=2))

    print(f"\n{'='*55}")
    print(f"✅  Done!  Captured:{captured}  Skipped:{skipped}")
    print(f"    Duration: {duration//60}m {duration%60}s")
    print(f"    Output: {out}")
    print(f"{'='*55}")
