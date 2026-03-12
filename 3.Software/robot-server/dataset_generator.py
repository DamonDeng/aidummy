"""
dataset_generator.py — Auto-labeling dataset builder for robot arm pose estimation.

Strategy:
1. Capture background model (arm parked at rest)
2. Sweep arm through a grid of J1/J2/J3/J5 poses
3. At each pose: capture image + background subtraction → arm mask → bbox + tip
4. Save YOLO-format dataset: images/ labels/ masks/ metadata/

Run:
    python3 dataset_generator.py [--output-dir /path/to/dataset] [--poses N]
"""

import cv2
import numpy as np
import json
import time
import argparse
import requests
import subprocess
from pathlib import Path
from datetime import datetime

SERVER = "http://127.0.0.1:3001"
CAMERA = "Astra Pro HD Camera"

# ── Joint safe limits ─────────────────────────────────────────────────────────
LIMITS = dict(j1=(-45,45), j2=(-73,90), j3=(35,180), j4=(-180,180), j5=(-120,120), j6=(-720,720))

def safe(j1,j2,j3,j4=0,j5=0,j6=0):
    for v, (lo,hi) in zip([j1,j2,j3,j4,j5,j6], LIMITS.values()):
        if not (lo <= v <= hi): return False
    # Arm must be raised enough to avoid crashing into desk:
    # upper_arm_angle = j2 + j3 - 90; require >= -20 (slightly below horizontal is ok)
    if j2 + j3 - 90 < -20: return False
    return True

def move_j(j1,j2,j3,j4=0,j5=0,j6=0,speed=20):
    r = requests.post(f"{SERVER}/robot/move_j",
                      json=dict(j1=j1,j2=j2,j3=j3,j4=j4,j5=j5,j6=j6,speed=speed))
    return r.json().get("ok", False)

def capture(path, warmup=1.5):
    """Capture from Astra Pro RGB camera."""
    subprocess.run(
        ["/opt/homebrew/bin/imagesnap", "-d", CAMERA, "-w", str(warmup), str(path)],
        capture_output=True
    )
    return Path(path).exists()

def build_background(out_path="/tmp/bg_model.npy", n_frames=15):
    """Park arm, sweep a few poses, compute median background."""
    print(f"\n{'='*55}")
    print("Step 1: Building background model")
    print(f"{'='*55}")

    # Park arm
    move_j(0,-73,180, speed=15)
    time.sleep(5)

    bg_poses = [
        (0,-73,180,0,0,0),
        (30,-60,160,0,0,0), (-30,-60,160,0,0,0),
        (40,-40,140,0,20,0), (-40,-40,140,0,-20,0),
        (0,-20,120,0,30,0),  (20,-20,120,0,-30,0),
        (0,-30,140,0,0,0),   (-20,-10,100,0,30,0),
        (20,-10,100,0,-30,0),(0,0,90,0,0,0),
        (35,-50,150,0,0,0),  (-35,-50,150,0,0,0),
        (0,-60,170,0,0,0),   (25,-30,130,0,0,0),
    ]

    frames = []
    tmp_dir = Path("/tmp/bg_frames_gen")
    tmp_dir.mkdir(exist_ok=True)

    for i, pose in enumerate(bg_poses[:n_frames]):
        j1,j2,j3,j4,j5,j6 = pose
        if not safe(j1,j2,j3,j4,j5,j6):
            print(f"  [{i+1}/{n_frames}] Pose {pose} unsafe, skipping")
            continue
        move_j(j1,j2,j3,j4,j5,j6, speed=20)
        time.sleep(3.5)
        p = tmp_dir / f"bg_{i:03d}.jpg"
        if capture(p):
            img = cv2.imread(str(p))
            if img is not None:
                frames.append(img.astype(np.float32))
                print(f"  [{len(frames)}/{n_frames}] Captured bg frame {i+1}")

    if len(frames) < 5:
        print("ERROR: Not enough background frames!")
        return None

    bg = np.median(np.stack(frames), axis=0).astype(np.uint8)
    np.save(out_path, bg)
    cv2.imwrite("/tmp/bg_preview.jpg", bg)
    print(f"✅ Background model: {bg.shape} from {len(frames)} frames → {out_path}")
    return bg

def arm_mask_from_diff(img, background, threshold=28):
    """Background subtraction → clean arm mask."""
    diff = np.abs(img.astype(np.float32) - background.astype(np.float32))
    diff_gray = diff.max(axis=2)
    mask = (diff_gray > threshold).astype(np.uint8) * 255

    k9  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9,9))
    k15 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k9,  iterations=3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k9,  iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k15, iterations=1)

    # Keep only the largest blob
    n, labels_img, stats, _ = cv2.connectedComponentsWithStats(mask)
    if n < 2:
        return None
    largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    clean = (labels_img == largest).astype(np.uint8) * 255
    # Require minimum size
    if stats[largest, cv2.CC_STAT_AREA] < 3000:
        return None
    return clean

def bbox_from_mask(mask):
    """YOLO normalized bbox [cx, cy, w, h] from binary mask."""
    H, W = mask.shape
    x,y,w,h = cv2.boundingRect(mask)
    return (x+w/2)/W, (y+h/2)/H, w/W, h/H

def tip_from_mask(mask):
    """Topmost arm pixel = tip candidate."""
    ys, xs = np.where(mask > 0)
    if len(ys) == 0:
        return None
    tip_y = int(ys.min())
    tip_x = int(xs[ys == tip_y].mean())
    return tip_x, tip_y

def generate_pose_grid():
    """Generate sweep grid: J1 × J2 × J3 × J5."""
    poses = []
    for j1 in range(-40, 45, 20):           # -40,-20,0,20,40
        for j2 in range(-60, 10, 20):        # -60,-40,-20,0
            for j3 in range(90, 175, 20):    # 90,110,130,150,170
                for j5 in (-30, 0, 30):      # wrist tilt variety
                    if safe(j1,j2,j3,0,j5,0):
                        poses.append((j1,j2,j3,0,j5,0))
    return poses

def run_dataset_generation(output_dir, background, max_poses=None):
    """Main sweep loop."""
    out = Path(output_dir)
    (out/"images").mkdir(parents=True, exist_ok=True)
    (out/"labels").mkdir(exist_ok=True)
    (out/"masks").mkdir(exist_ok=True)
    (out/"metadata").mkdir(exist_ok=True)

    poses = generate_pose_grid()
    if max_poses:
        poses = poses[:max_poses]

    H, W = background.shape[:2]
    print(f"\n{'='*55}")
    print(f"Step 2: Sweeping {len(poses)} poses")
    print(f"Output: {out}")
    print(f"Estimated time: ~{len(poses)*5//60} min {len(poses)*5%60} sec")
    print(f"{'='*55}\n")

    results = {"total": len(poses), "captured": 0, "skipped": 0, "poses": []}
    start_time = time.time()

    for idx, (j1,j2,j3,j4,j5,j6) in enumerate(poses):
        stem = f"pose_{idx:04d}_j{j1:+d}_{j2:+d}_{j3:+d}_{j5:+d}"
        img_path  = out/"images"/f"{stem}.jpg"
        lbl_path  = out/"labels"/f"{stem}.txt"
        mask_path = out/"masks"/f"{stem}.png"
        meta_path = out/"metadata"/f"{stem}.json"

        # Skip already done
        if img_path.exists() and lbl_path.exists():
            results["captured"] += 1
            continue

        elapsed = time.time() - start_time
        eta = (elapsed/(idx+1)) * (len(poses)-idx-1) if idx > 0 else 0
        print(f"[{idx+1:3d}/{len(poses)}] J1={j1:+3d} J2={j2:+3d} J3={j3:+3d} J5={j5:+3d}  "
              f"ETA:{eta/60:.1f}min", end="  ", flush=True)

        # Move
        move_j(j1,j2,j3,j4,j5,j6, speed=25)
        time.sleep(3.8)  # settle

        # Capture
        if not capture(img_path):
            print("CAPTURE FAIL")
            results["skipped"] += 1
            continue

        # Process
        img = cv2.imread(str(img_path))
        if img is None:
            print("READ FAIL")
            results["skipped"] += 1
            continue

        mask = arm_mask_from_diff(img, background)
        if mask is None:
            print("NO MASK")
            results["skipped"] += 1
            continue

        cx,cy,bw,bh = bbox_from_mask(mask)
        tip = tip_from_mask(mask)
        arm_pixels = int(np.sum(mask > 0))

        # YOLO label: class 0 = robot_arm, bbox only for now
        # Format: class cx cy w h
        lbl_path.write_text(f"0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")

        # Save mask
        cv2.imwrite(str(mask_path), mask)

        # Save metadata
        meta = {
            "idx": idx, "stem": stem,
            "joints": {"j1":j1,"j2":j2,"j3":j3,"j4":j4,"j5":j5,"j6":j6},
            "bbox_yolo": [cx,cy,bw,bh],
            "tip_pixel": tip,
            "arm_pixels": arm_pixels,
            "timestamp": datetime.now().isoformat()
        }
        meta_path.write_text(json.dumps(meta, indent=2))

        results["captured"] += 1
        print(f"✓  pixels={arm_pixels}  tip={tip}")

    # Park arm when done
    move_j(0,-73,180, speed=15)

    # Save summary
    results["duration_sec"] = int(time.time()-start_time)
    (out/"dataset_summary.json").write_text(json.dumps(results, indent=2))

    print(f"\n{'='*55}")
    print(f"✅ Done! Captured: {results['captured']}  Skipped: {results['skipped']}")
    print(f"   Duration: {results['duration_sec']//60}m {results['duration_sec']%60}s")
    print(f"   Dataset at: {out}")
    print(f"{'='*55}")
    return results

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--output-dir", default="/tmp/arm_dataset")
    ap.add_argument("--poses", type=int, default=None, help="Limit number of poses")
    ap.add_argument("--skip-background", action="store_true")
    ap.add_argument("--bg-model", default="/tmp/bg_model.npy")
    args = ap.parse_args()

    # Check server
    try:
        status = requests.get(f"{SERVER}/robot/status").json()
        print(f"Robot status: {status}")
        if status.get("status") != "connected":
            print("ERROR: Robot not connected"); exit(1)
    except Exception as e:
        print(f"ERROR: Cannot reach server: {e}"); exit(1)

    # Background
    bg_path = Path(args.bg_model)
    if args.skip_background and bg_path.exists():
        print(f"Loading existing background from {bg_path}")
        background = np.load(bg_path)
    else:
        background = build_background(str(bg_path))
        if background is None: exit(1)

    # Generate dataset
    run_dataset_generation(args.output_dir, background, max_poses=args.poses)
