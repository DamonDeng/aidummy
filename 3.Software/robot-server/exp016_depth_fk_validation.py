"""
EXP-016 — Depth Camera vs FK Tip Position Validation

Strategy:
  1. Move arm tip to N known positions (joint angles → FK gives tip in robot frame, mm)
  2. At each position, capture a depth frame
  3. Background-subtract to isolate arm pixels in the depth image
  4. Find the closest pixel cluster = arm tip in camera frame (u, v, depth_mm)
  5. Convert (u,v,d) → 3D point in camera frame using Astra Pro depth intrinsics
  6. Record pairs: (robot_frame_tip, camera_frame_tip)
  7. Compute the best-fit rigid transform R,t (camera → robot)
  8. Report reprojection error — this validates depth accuracy

Astra Pro depth camera intrinsics (typical, 640×480):
  fx = fy = 570.34
  cx = 319.5, cy = 239.5
"""

import sys
import os
import time
import math
import json
import numpy as np
import requests

# ─── Setup ─────────────────────────────────────────────────────────────────────
SERVER = "http://127.0.0.1:3001"

# Add robot-server dir to path for ik_solver + depth_driver
sys.path.insert(0, os.path.dirname(__file__))

from ik_solver import (
    _euler_to_rot, _mat_mul_3x3_3x1, _mat_mul_3x3_3x3,
    _rot_to_euler,
    L_BS, D_BS, L_AM, L_FA, D_EW, L_WT, L1_BASE, L6_WRIST,
)
from depth_driver import DepthDriver

# ─── FK (forward kinematics) ───────────────────────────────────────────────────

def fk(joints_deg):
    """Return tip position (x, y, z) in robot frame, mm."""
    DH = [
        [0.0,        L_BS, D_BS, -math.pi/2],
        [-math.pi/2, 0,    L_AM,  0],
        [ math.pi/2, D_EW, 0,     math.pi/2],
        [0,          L_FA, 0,    -math.pi/2],
        [0,          0,    0,     math.pi/2],
        [0,          L_WT, 0,     0],
    ]
    def rot(theta, alpha):
        c, s   = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)
        return [c, -ca*s, sa*s, s, ca*c, -sa*c, 0, sa, ca]

    Rs = [rot(math.radians(j) + DH[i][0], DH[i][3]) for i, j in enumerate(joints_deg)]
    R02 = _mat_mul_3x3_3x3(Rs[0], Rs[1])
    R03 = _mat_mul_3x3_3x3(R02, Rs[2])
    R06 = _mat_mul_3x3_3x3(R03, _mat_mul_3x3_3x3(Rs[3], _mat_mul_3x3_3x3(Rs[4], Rs[5])))
    pos_m = [
        sum(Rs[0][i*3+j]*L1_BASE[j] for j in range(3)) +
        sum(R02[i*3+j]*[L_AM, 0, 0][j] for j in range(3)) +
        sum(R03[i*3+j]*[-D_EW, 0, L_FA][j] for j in range(3)) +
        sum(R06[i*3+j]*L6_WRIST[j] for j in range(3))
        for i in range(3)
    ]
    return [p * 1000.0 for p in pos_m]  # → mm


# ─── Depth camera intrinsics ───────────────────────────────────────────────────
# Astra Pro depth module typical values for 640×480
FX = FY = 570.34
CX, CY = 319.5, 239.5


def pixel_to_3d(u, v, depth_mm):
    """Unproject pixel (u,v) with depth to 3D point in camera frame (mm)."""
    z = float(depth_mm)
    x = (u - CX) * z / FX
    y = (v - CY) * z / FY
    return np.array([x, y, z])


# ─── Arm motion helpers ─────────────────────────────────────────────────────────

def move_to(joints, speed=20, settle_threshold=1.0, timeout=15):
    r = requests.post(f"{SERVER}/robot/play_sequence", json={
        "speed": speed,
        "settle_threshold": settle_threshold,
        "settle_timeout": timeout,
        "steps": [{"move_j": joints}]
    })
    r.raise_for_status()
    return r.json()


def get_angles():
    return list(requests.get(f"{SERVER}/robot/angles").json().values())


# ─── Depth tip detection ────────────────────────────────────────────────────────

def capture_depth_raw(driver):
    """Capture one depth frame. Returns (H×W uint16 array, scale_mm)."""
    raw, W, H, scale = driver.capture()
    return raw.reshape(H, W), scale


def find_arm_centroid(bg_frame, fg_frame, scale,
                      diff_thresh_mm=30, depth_min_mm=120, depth_max_mm=800,
                      min_cluster_px=50, debug=False):
    """
    Background-subtract to find arm pixels, then return the 3D centroid of
    the filtered arm cluster in camera frame.

    depth_min_mm / depth_max_mm: ignore pixels outside this range
      (removes close-range sensor artifacts and far-field noise)

    Returns (u_centroid, v_centroid, depth_centroid_mm, 3d_centroid_mm, n_pixels)
    or None if no sufficient cluster found.
    """
    H, W = bg_frame.shape

    bg_mm = bg_frame.astype(np.float32) * scale
    fg_mm = fg_frame.astype(np.float32) * scale

    bg_valid = bg_frame > 0
    fg_valid = fg_frame > 0

    # Arm pixels: foreground significantly closer than background, or new pixels
    arm_raw = (
        (fg_valid & bg_valid & ((bg_mm - fg_mm) > diff_thresh_mm)) |
        (fg_valid & ~bg_valid)
    )

    # Apply depth range filter to reject artifacts and far-field noise
    depth_ok  = (fg_mm >= depth_min_mm) & (fg_mm <= depth_max_mm)
    arm_mask  = arm_raw & depth_ok

    n_pixels = int(arm_mask.sum())
    if n_pixels < min_cluster_px:
        if debug:
            raw_n = int(arm_raw.sum())
            print(f"  [arm_centroid] {n_pixels} arm pixels after depth filter "
                  f"(raw={raw_n}, range={depth_min_mm}-{depth_max_mm}mm) — skipping")
        return None

    # Pixel centroid (u, v)
    vs, us = np.where(arm_mask)
    u_c = float(us.mean())
    v_c = float(vs.mean())

    # Depth centroid (median is more robust than mean against outliers)
    d_c = float(np.median(fg_mm[arm_mask]))

    # 3D centroid via average of unprojected arm pixels
    depths = fg_mm[arm_mask]
    xs = (us - CX) * depths / FX
    ys = (vs - CY) * depths / FY
    pt3d = np.array([xs.mean(), ys.mean(), depths.mean()])

    if debug:
        d_range = (fg_mm[arm_mask].min(), fg_mm[arm_mask].max())
        print(f"  [arm_centroid] {n_pixels} px | "
              f"u={u_c:.0f} v={v_c:.0f} | depth {d_range[0]:.0f}-{d_range[1]:.0f}mm "
              f"median={d_c:.0f}mm | 3D=({pt3d[0]:.1f},{pt3d[1]:.1f},{pt3d[2]:.1f})mm")

    return u_c, v_c, d_c, pt3d, n_pixels


# ─── Rigid transform solver ─────────────────────────────────────────────────────

def solve_rigid_transform(cam_pts, rob_pts):
    """
    Solve R, t such that  rob_pt ≈ R @ cam_pt + t
    (camera frame → robot frame)

    Uses SVD-based Umeyama/Horn method.
    Returns (R 3×3, t 3×1, residuals_mm).
    """
    cam = np.array(cam_pts, dtype=np.float64)   # N×3
    rob = np.array(rob_pts, dtype=np.float64)   # N×3

    mu_c = cam.mean(axis=0)
    mu_r = rob.mean(axis=0)

    Ac = cam - mu_c
    Ar = rob - mu_r

    H = Ac.T @ Ar   # 3×3
    U, S, Vt = np.linalg.svd(H)

    # Ensure right-handed coordinate system
    D = np.diag([1, 1, np.linalg.det(Vt.T @ U.T)])
    R = Vt.T @ D @ U.T

    t = mu_r - R @ mu_c

    # Per-point residuals
    predicted = (R @ cam.T).T + t
    residuals = np.linalg.norm(predicted - rob, axis=1)

    return R, t, residuals


# ─── Main experiment ────────────────────────────────────────────────────────────

# Test poses: (label, [j1, j2, j3, j4, j5, j6])
# Chosen to spread the tip across different (x, y, z) positions visible to camera
TEST_POSES = [
    ("center_mid",  [ 0,  10,  85, 0, 0, 0]),
    ("left_mid",    [-30, 10,  85, 0, 0, 0]),
    ("right_mid",   [ 30, 10,  85, 0, 0, 0]),
    ("center_high", [ 0,  30,  75, 0, 0, 0]),
    ("center_low",  [ 0,  -5,  95, 0, 0, 0]),
    ("left_high",   [-25, 25,  75, 0, 0, 0]),
    ("right_high",  [ 25, 25,  75, 0, 0, 0]),
]

# Background pose: arm folded away from camera FOV
BG_POSE = [0, -73, 180, 0, 0, 0]  # rest pose

def run():
    print("=" * 60)
    print("EXP-016: Depth Camera vs FK Tip Position Validation")
    print("=" * 60)

    results = []
    cam_pts = []
    rob_pts = []

    print("\n[1/4] Opening depth driver...")
    with DepthDriver(width=640, height=480, fps=30) as driver:

        print("[2/4] Capturing background frame (arm at rest)...")
        print(f"  Moving to BG pose {BG_POSE}")
        move_to(BG_POSE, speed=20)
        time.sleep(1.0)
        bg_frame, scale = capture_depth_raw(driver)
        print(f"  Background captured. scale={scale}mm/unit")

        print(f"\n[3/4] Running {len(TEST_POSES)} test poses...\n")

        for label, joints in TEST_POSES:
            print(f"  Pose: {label} → joints={joints}")

            # FK tip position in robot frame
            tip_robot = fk(joints)
            print(f"    FK tip (robot frame): x={tip_robot[0]:.1f} y={tip_robot[1]:.1f} z={tip_robot[2]:.1f} mm")

            # Move arm
            result = move_to(joints, speed=25, settle_threshold=1.0)
            settle_s = result["step_log"][0]["settle_s"]
            print(f"    Settled in {settle_s:.2f}s")

            # Capture depth
            fg_frame, _ = capture_depth_raw(driver)

            # Detect arm centroid
            det = find_arm_centroid(bg_frame, fg_frame, scale, debug=True)

            if det is None:
                print(f"    ⚠️  Arm not detected in depth frame — skipping\n")
                results.append({
                    "label": label, "joints": joints,
                    "fk_robot_mm": tip_robot,
                    "detected": False,
                })
                continue

            u_c, v_c, d_c, pt3d_cam, npx = det
            print(f"    Camera 3D centroid: X={pt3d_cam[0]:.1f} Y={pt3d_cam[1]:.1f} Z={pt3d_cam[2]:.1f} mm")

            cam_pts.append(pt3d_cam.tolist())
            rob_pts.append(tip_robot)

            results.append({
                "label": label,
                "joints": joints,
                "fk_robot_mm": [round(v, 1) for v in tip_robot],
                "cam_uv_centroid": [round(u_c, 1), round(v_c, 1)],
                "cam_depth_median_mm": round(d_c, 1),
                "cam_3d_centroid_mm": [round(float(x), 1) for x in pt3d_cam],
                "n_arm_pixels": npx,
                "detected": True,
            })
            print()

        # Return to rest
        print("  Returning to rest pose...")
        move_to(BG_POSE, speed=15)

    # ── Solve camera-to-robot transform ──────────────────────────────────────
    print(f"\n[4/4] Solving camera → robot rigid transform ({len(cam_pts)} point pairs)...")

    transform_result = None
    if len(cam_pts) >= 3:
        R, t, residuals = solve_rigid_transform(cam_pts, rob_pts)
        transform_result = {
            "R": R.tolist(),
            "t": t.tolist(),
            "residuals_mm": [round(float(r), 1) for r in residuals],
            "mean_error_mm": round(float(residuals.mean()), 1),
            "max_error_mm":  round(float(residuals.max()), 1),
        }
        print(f"\n  Camera → Robot transform:")
        print(f"  R =\n{np.round(R, 4)}")
        print(f"  t = {np.round(t, 1)} mm")
        print(f"  Residuals: {[round(float(r),1) for r in residuals]} mm")
        print(f"  Mean error: {residuals.mean():.1f} mm")
        print(f"  Max  error: {residuals.max():.1f} mm")
    else:
        print(f"  Not enough detected points ({len(cam_pts)}) — need ≥3 for transform")

    # ── Save results ──────────────────────────────────────────────────────────
    out = {
        "experiment": "EXP-016",
        "intrinsics": {"fx": FX, "fy": FY, "cx": CX, "cy": CY},
        "n_poses": len(TEST_POSES),
        "n_detected": len(cam_pts),
        "poses": results,
        "transform": transform_result,
    }
    out_path = os.path.join(os.path.dirname(__file__), "..", "..", "experiment_logs", "EXP-016-results.json")
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\n  Results saved to {out_path}")
    print("\n✅ EXP-016 complete.")
    return out


if __name__ == "__main__":
    run()
