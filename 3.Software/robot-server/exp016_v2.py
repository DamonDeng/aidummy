"""
EXP-016 v2 — Inter-pose variance approach to find arm tip region

Instead of centroid of whole arm body (barely moves), find pixels that
VARY MOST across all poses = the distal/tip end of the arm.
Then use those high-variance pixels per-pose to get tip position.
"""
import sys, os, time, math, json, numpy as np, requests
sys.path.insert(0, os.path.dirname(__file__))

from depth_driver import DepthDriver
from ik_solver import (_mat_mul_3x3_3x3, _rot_to_euler,
                       L_BS, D_BS, L_AM, L_FA, D_EW, L_WT, L1_BASE, L6_WRIST)

SERVER = "http://127.0.0.1:3001"
FX = FY = 570.34; CX, CY = 319.5, 239.5

def fk(j):
    DH = [[0,L_BS,D_BS,-math.pi/2],[-math.pi/2,0,L_AM,0],[math.pi/2,D_EW,0,math.pi/2],
          [0,L_FA,0,-math.pi/2],[0,0,0,math.pi/2],[0,L_WT,0,0]]
    def rot(t,a):
        c,s,ca,sa=math.cos(t),math.sin(t),math.cos(a),math.sin(a)
        return[c,-ca*s,sa*s,s,ca*c,-sa*c,0,sa,ca]
    Rs=[rot(math.radians(j[i])+DH[i][0],DH[i][3]) for i in range(6)]
    R02=_mat_mul_3x3_3x3(Rs[0],Rs[1]); R03=_mat_mul_3x3_3x3(R02,Rs[2])
    R06=_mat_mul_3x3_3x3(R03,_mat_mul_3x3_3x3(Rs[3],_mat_mul_3x3_3x3(Rs[4],Rs[5])))
    pos=[sum(Rs[0][i*3+k]*L1_BASE[k] for k in range(3))+
         sum(R02[i*3+k]*[L_AM,0,0][k] for k in range(3))+
         sum(R03[i*3+k]*[-D_EW,0,L_FA][k] for k in range(3))+
         sum(R06[i*3+k]*L6_WRIST[k] for k in range(3)) for i in range(3)]
    return [p*1000 for p in pos]

def move_to(joints, speed=25):
    r = requests.post(f"{SERVER}/robot/play_sequence", json={
        "speed": speed, "settle_threshold": 1.0, "settle_timeout": 15,
        "steps": [{"move_j": joints}]
    })
    r.raise_for_status()
    return r.json()

def solve_rigid_transform(cam_pts, rob_pts):
    cam = np.array(cam_pts, dtype=np.float64)
    rob = np.array(rob_pts, dtype=np.float64)
    mu_c, mu_r = cam.mean(0), rob.mean(0)
    H = (cam - mu_c).T @ (rob - mu_r)
    U,S,Vt = np.linalg.svd(H)
    D = np.diag([1,1,np.linalg.det(Vt.T@U.T)])
    R = Vt.T @ D @ U.T
    t = mu_r - R @ mu_c
    predicted = (R @ cam.T).T + t
    residuals = np.linalg.norm(predicted - rob, axis=1)
    return R, t, residuals

POSES = [
    ("center_mid",  [ 0,  10,  85, 0, 0, 0]),
    ("left_mid",    [-30, 10,  85, 0, 0, 0]),
    ("right_mid",   [ 30, 10,  85, 0, 0, 0]),
    ("center_high", [ 0,  30,  75, 0, 0, 0]),
    ("center_low",  [ 0,  -5,  95, 0, 0, 0]),
    ("left_high",   [-25, 25,  75, 0, 0, 0]),
    ("right_high",  [ 25, 25,  75, 0, 0, 0]),
]
BG = [0, -73, 180, 0, 0, 0]

print("="*60)
print("EXP-016 v2: Inter-pose variance tip detection")
print("="*60)

frames = {}
fk_pos = {}
print("\n[1/4] Capturing frames...")
with DepthDriver(width=640, height=480, fps=30) as d:
    move_to(BG, speed=15)
    time.sleep(1)
    raw,W,H,sc = d.capture()
    bg = raw.reshape(H,W).astype(np.float32)*sc
    print(f"  Background captured (scale={sc}mm/unit)")

    for label, joints in POSES:
        r = move_to(joints)
        s = r["step_log"][0]["settle_s"]
        raw,W,H,sc2 = d.capture()
        frames[label] = raw.reshape(H,W).astype(np.float32)*sc2
        fk_pos[label] = fk(joints)
        p = fk_pos[label]
        print(f"  {label}: {s:.1f}s  FK=({p[0]:.0f},{p[1]:.0f},{p[2]:.0f})mm")

    move_to(BG, speed=15)
    print("  Returned to rest.")

print("\n[2/4] Computing inter-pose variance map...")
stack = np.stack(list(frames.values()), axis=0)  # (N,H,W)
valid = (stack >= 120) & (stack <= 800)
masked = np.where(valid, stack, np.nan)
variance_map = np.nanvar(masked, axis=0)   # high-var pixels = most movement = TIP

# Top 10% variance pixels = tip region
var_vals = variance_map[variance_map > 0]
var_thresh = np.percentile(var_vals, 90)
tip_mask = variance_map >= var_thresh
print(f"  Variance threshold (p90): {var_thresh:.0f}")
print(f"  Tip-region pixels: {tip_mask.sum()}")
vs_t, us_t = np.where(tip_mask)
print(f"  Tip region spans: u={us_t.min()}-{us_t.max()} v={vs_t.min()}-{vs_t.max()}")

print("\n[3/4] Per-pose tip centroid (high-variance pixels only)...")
cam_pts = []; rob_pts = []
rows = []
for label, joints in POSES:
    fg = frames[label]
    m = tip_mask & (fg >= 120) & (fg <= 800)
    if m.sum() < 10:
        print(f"  {label}: not enough pixels ({m.sum()})")
        continue
    vs, us = np.where(m)
    depths = fg[m]
    xs = (us - CX)*depths/FX
    ys = (vs - CY)*depths/FY
    pt = np.array([xs.mean(), ys.mean(), depths.mean()])
    cam_pts.append(pt.tolist())
    rob_pts.append(fk_pos[label])
    p = fk_pos[label]
    rows.append((label, p, pt, int(m.sum())))
    print(f"  {label:<14}  FK=({p[0]:>6.0f},{p[1]:>6.0f},{p[2]:>6.0f})  "
          f"cam=({pt[0]:>6.1f},{pt[1]:>6.1f},{pt[2]:>6.1f})  n={m.sum()}")

print(f"\n[4/4] Rigid transform ({len(cam_pts)} pairs)...")
R, t, res = solve_rigid_transform(cam_pts, rob_pts)
print(f"  Residuals: {[round(float(r),1) for r in res]} mm")
print(f"  Mean: {res.mean():.1f} mm  Max: {res.max():.1f} mm")
print(f"  R =\n{np.round(R,4)}")
print(f"  t = {np.round(t,1)} mm")

# --- Motion correlation check ---
cam = np.array(cam_pts); rob = np.array(rob_pts)
cam_c = cam - cam.mean(0); rob_c = rob - rob.mean(0)
print("\n  Motion correlation (centered):")
print(f"  {'Pose':<14}  FK Δy(mm)  CamΔx(mm)  FK Δz(mm)  CamΔy(mm)")
for i,(label,_) in enumerate(POSES):
    if i < len(cam_c):
        print(f"  {label:<14}  {rob_c[i,1]:>+9.0f}  {cam_c[i,0]:>+9.1f}  "
              f"{rob_c[i,2]:>+9.0f}  {cam_c[i,1]:>+9.1f}")

# Save
out = {
    "experiment": "EXP-016v2",
    "intrinsics": {"fx":FX,"fy":FY,"cx":CX,"cy":CY},
    "variance_threshold": float(var_thresh),
    "n_detected": len(cam_pts),
    "poses": [{"label":r[0],"fk_robot_mm":[round(v,1) for v in r[1]],
               "cam_3d_tip_mm":[round(float(x),1) for x in r[2]],"n_pixels":r[3]}
              for r in rows],
    "transform": {"R":R.tolist(),"t":t.tolist(),
                  "residuals_mm":[round(float(r),1) for r in res],
                  "mean_error_mm":round(float(res.mean()),1),
                  "max_error_mm":round(float(res.max()),1)},
}
out_path = os.path.join(os.path.dirname(__file__), "../../experiment_logs/EXP-016-results-v2.json")
with open(out_path,"w") as f: json.dump(out,f,indent=2)
print(f"\n  Results → {out_path}")
print("\n✅ EXP-016 v2 complete.")
