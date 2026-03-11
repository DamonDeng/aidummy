"""
EXP-016 v3 — Consecutive-frame depth difference to isolate arm tip

Key insight from v1/v2:
- J1 sweep moves tip LATERALLY in the image (big u-pixel change: ~590px for ±30°)
- Depth barely changes because the camera faces the arm frontally
- Centroid of whole arm body doesn't work (arm fills frame)
- Variance-map doesn't work (background exposure causes false high-variance)

New approach: pairwise frame difference
- For each pair (frame_A, frame_B), compute |depth_A - depth_B| where both valid
- High-diff pixels = where the arm ACTUALLY MOVED between these two poses = TIP region
- In frame_A, those high-diff pixels that are SHALLOWER than frame_B = arm moved TOWARD us
- Report the centroid of those pixels in 3D for each pose pair
"""
import sys, os, time, math, json, numpy as np, requests
sys.path.insert(0, os.path.dirname(__file__))
from depth_driver import DepthDriver
from ik_solver import (_mat_mul_3x3_3x3, L_BS, D_BS, L_AM, L_FA, D_EW, L_WT, L1_BASE, L6_WRIST)

SERVER = "http://127.0.0.1:3001"
FX = FY = 570.34; CX, CY = 319.5, 239.5
DEPTH_MIN, DEPTH_MAX = 100, 900

def fk(j):
    DH = [[0,L_BS,D_BS,-math.pi/2],[-math.pi/2,0,L_AM,0],[math.pi/2,D_EW,0,math.pi/2],
          [0,L_FA,0,-math.pi/2],[0,0,0,math.pi/2],[0,L_WT,0,0]]
    def rot(t,a):
        c,s,ca,sa=math.cos(t),math.sin(t),math.cos(a),math.sin(a)
        return[c,-ca*s,sa*s,s,ca*c,-sa*c,0,sa,ca]
    Rs=[rot(math.radians(j[i])+DH[i][0],DH[i][3]) for i in range(6)]
    R02=_mat_mul_3x3_3x3(Rs[0],Rs[1]); R03=_mat_mul_3x3_3x3(R02,Rs[2])
    R06=_mat_mul_3x3_3x3(R03,_mat_mul_3x3_3x3(Rs[3],_mat_mul_3x3_3x3(Rs[4],Rs[5])))
    p=[sum(Rs[0][i*3+k]*L1_BASE[k] for k in range(3))+sum(R02[i*3+k]*[L_AM,0,0][k] for k in range(3))+
       sum(R03[i*3+k]*[-D_EW,0,L_FA][k] for k in range(3))+sum(R06[i*3+k]*L6_WRIST[k] for k in range(3)) for i in range(3)]
    return [x*1000 for x in p]

def move_to(j, speed=25):
    r = requests.post(f"{SERVER}/robot/play_sequence", json={
        "speed": speed, "settle_threshold": 1.0, "settle_timeout": 15,
        "steps": [{"move_j": j}]
    })
    r.raise_for_status(); return r.json()

def solve_rigid(cam_pts, rob_pts):
    cam=np.array(cam_pts); rob=np.array(rob_pts)
    mu_c,mu_r=cam.mean(0),rob.mean(0)
    H=(cam-mu_c).T@(rob-mu_r)
    U,S,Vt=np.linalg.svd(H)
    D=np.diag([1,1,np.linalg.det(Vt.T@U.T)])
    R=Vt.T@D@U.T; t=mu_r-R@mu_c
    res=np.linalg.norm((R@cam.T).T+t-rob,axis=1)
    return R,t,res

# Poses — J1 varied widely for lateral spread, J2/J3 varied for depth spread
POSES = [
    ("center",   [ 0,  10, 85, 0, 0, 0]),
    ("left30",   [-30, 10, 85, 0, 0, 0]),
    ("right30",  [ 30, 10, 85, 0, 0, 0]),
    ("high",     [ 0,  30, 75, 0, 0, 0]),
    ("low",      [ 0,  -5, 95, 0, 0, 0]),
    ("left_high",[-25, 25, 75, 0, 0, 0]),
    ("right_high",[ 25, 25, 75, 0, 0, 0]),
]
BG = [0, -73, 180, 0, 0, 0]

print("="*60); print("EXP-016 v3: Pairwise depth-diff tip isolation"); print("="*60)

# ── 1. Capture all frames ─────────────────────────────────────────────────────
frames = {}; fk_pos = {}
print("\n[1/4] Capturing frames...")
with DepthDriver(width=640, height=480, fps=30) as d:
    move_to(BG, speed=15); time.sleep(1)
    raw,W,H,sc=d.capture(); bg=raw.reshape(H,W).astype(np.float32)*sc
    print(f"  Background at rest (scale={sc}mm/unit)")
    for label, joints in POSES:
        r=move_to(joints); s=r["step_log"][0]["settle_s"]
        raw,W,H,sc2=d.capture()
        frames[label]=raw.reshape(H,W).astype(np.float32)*sc2
        fk_pos[label]=fk(joints)
        p=fk_pos[label]
        print(f"  {label:<12}: {s:.1f}s settle  FK=({p[0]:.0f},{p[1]:.0f},{p[2]:.0f})mm")
    move_to(BG,speed=15); print("  Returned to rest.")

# ── 2. Background subtraction to get arm masks ────────────────────────────────
print("\n[2/4] Building arm masks (foreground vs background)...")
arm_masks = {}
for label, _ in POSES:
    fg = frames[label]
    bg_valid = bg > 0; fg_valid = fg > 0
    bg_mm = bg; fg_mm = fg
    # Arm: foreground significantly closer than background, or new pixels
    m = ((fg_valid & bg_valid & ((bg_mm - fg_mm) > 20)) | (fg_valid & ~bg_valid))
    m &= (fg >= DEPTH_MIN) & (fg <= DEPTH_MAX)
    arm_masks[label] = m
    vs, us = np.where(m)
    print(f"  {label:<12}: {m.sum()} arm px  u={us.min() if m.sum()>0 else '?'}-{us.max() if m.sum()>0 else '?'}  "
          f"v={vs.min() if m.sum()>0 else '?'}-{vs.max() if m.sum()>0 else '?'}")

# ── 3. Pairwise difference: find TIP pixels ──────────────────────────────────
# Use LEFT vs RIGHT pairs to find the laterally-moving tip
# For a left pose: the tip is the ARM pixels that DISAPPEARED when going to the right pose
print("\n[3/4] Pairwise difference to isolate tip pixels...")

# Key pairs: opposite J1 poses should show maximum tip movement
PAIRS = [
    ("left30",  "right30",  "lateral_mid"),
    ("left_high","right_high","lateral_high"),
    ("center",  "high",     "vertical_center"),
    ("center",  "low",      "vertical_low"),
]

tip_results = {}  # label → 3D point in camera frame

for label_a, label_b, pair_name in PAIRS:
    fa = frames[label_a]; fb = frames[label_b]
    ma = arm_masks[label_a]; mb = arm_masks[label_b]

    # Pixels where arm is in A but not B = tip region for pose A
    tip_a = ma & ~mb & (fa >= DEPTH_MIN) & (fa <= DEPTH_MAX)
    # Pixels where arm is in B but not A = tip region for pose B
    tip_b = mb & ~ma & (fb >= DEPTH_MIN) & (fb <= DEPTH_MAX)

    print(f"\n  Pair {label_a} vs {label_b}:")
    for lbl, tip_mask, frame in [(label_a, tip_a, fa), (label_b, tip_b, fb)]:
        n = int(tip_mask.sum())
        if n < 5:
            print(f"    {lbl}: only {n} tip pixels — not enough")
            continue
        vs, us = np.where(tip_mask)
        depths = frame[tip_mask]
        xs = (us - CX)*depths/FX; ys = (vs - CY)*depths/FY
        pt = np.array([xs.mean(), ys.mean(), depths.mean()])
        # Also report pixel centroid
        u_c, v_c = us.mean(), vs.mean()
        print(f"    {lbl:<12}: {n:>5}px  pix=({u_c:.0f},{v_c:.0f})  "
              f"depth={depths.mean():.0f}mm  3D=({pt[0]:.1f},{pt[1]:.1f},{pt[2]:.1f})mm")
        # Store best (most pixels) estimate for this label
        if lbl not in tip_results or n > tip_results[lbl][1]:
            tip_results[lbl] = (pt, n, u_c, v_c, depths.mean())

# ── 4. Assemble point pairs and solve transform ───────────────────────────────
print("\n[4/4] Assembling matched pairs and solving transform...")
cam_pts = []; rob_pts = []
print(f"  {'Pose':<12}  {'FK (x,y,z)':<28}  {'Cam 3D (x,y,z)':<28}  n")
print("  "+"-"*75)
for label, joints in POSES:
    if label not in tip_results:
        continue
    pt, n, u_c, v_c, d = tip_results[label]
    p = fk_pos[label]
    cam_pts.append(pt.tolist()); rob_pts.append(p)
    print(f"  {label:<12}  ({p[0]:>6.0f},{p[1]:>6.0f},{p[2]:>6.0f})  "
          f"({pt[0]:>7.1f},{pt[1]:>7.1f},{pt[2]:>7.1f})  pix=({u_c:.0f},{v_c:.0f})  n={n}")

if len(cam_pts) >= 3:
    R, t, res = solve_rigid(cam_pts, rob_pts)
    print(f"\n  Residuals: {[round(float(r),1) for r in res]} mm")
    print(f"  Mean error: {res.mean():.1f} mm  |  Max: {res.max():.1f} mm")
    print(f"  R =\n{np.round(R,4)}")
    print(f"  t = {np.round(t,1)} mm")

    # Sanity check: FK Δy should correlate with cam ΔX
    cam=np.array(cam_pts); rob=np.array(rob_pts)
    print("\n  Correlation check (centered deltas):")
    print(f"  {'Pose':<12}  FK Δy  →  Cam ΔX   |  FK Δz  →  Cam ΔY")
    mc=cam.mean(0); mr=rob.mean(0)
    for i,(l,_) in enumerate([(l,j) for l,j in POSES if l in tip_results]):
        print(f"  {l:<12}  {rob[i,1]-mr[1]:>+7.0f}  →  {cam[i,0]-mc[0]:>+7.1f}   |  "
              f"{rob[i,2]-mr[2]:>+7.0f}  →  {cam[i,1]-mc[1]:>+7.1f}")
else:
    print(f"  Only {len(cam_pts)} valid pairs — need ≥3")

out={
    "experiment":"EXP-016v3",
    "tip_results":{k:{"cam_3d":v[0].tolist(),"n_pixels":v[1],"pixel_uv":[float(v[2]),float(v[3])],"depth_mm":float(v[4])}
                   for k,v in tip_results.items()},
    "fk_positions":{k:[round(x,1) for x in v] for k,v in fk_pos.items()},
}
out_path=os.path.join(os.path.dirname(__file__),"../../experiment_logs/EXP-016-results-v3.json")
with open(out_path,"w") as f: json.dump(out,f,indent=2)
print(f"\n  Saved → {out_path}")
print("\n✅ EXP-016 v3 complete.")
