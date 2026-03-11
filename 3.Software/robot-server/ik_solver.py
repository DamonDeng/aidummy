"""
Python IK solver — exact mirror of the firmware's DOF6Kinematic::SolveIK.

DH parameters (metres):
  L_BS=0.109, D_BS=0.035, L_AM=0.146, L_FA=0.115, D_EW=0.052, L_WT=0.072

Euler convention (firmware EulerAngleToRotMat):
  R = Rz(c) @ Ry(b) @ Rx(a)   — a,b,c in DEGREES as passed to the API.
  (The firmware variable naming uses cc/sc for cos/sin of 'a', ca/sa for 'c' —
   confusing internally, but the resulting matrix and convention are confirmed.)

Input units: x,y,z in MILLIMETRES, a,b,c in DEGREES (same as the move_l API).
Output: list of up to 8 joint configs in DEGREES (physical angles).
"""

import math
from typing import Optional

# ── DH / link parameters (metres) ─────────────────────────────────────────────
L_BS = 0.109
D_BS = 0.035
L_AM = 0.146
L_FA = 0.115
D_EW = 0.052
L_WT = 0.072

# DH home offsets [theta0_rad] per joint
DH_THETA = [0.0, -math.pi / 2, math.pi / 2, 0.0, 0.0, 0.0]

# Link vectors (from firmware constructor)
L1_BASE  = [D_BS, -L_BS, 0.0]
L6_WRIST = [0.0,  0.0,   L_WT]

# Squared / cached lengths
_l_se   = L_AM
_l_se_2 = L_AM * L_AM
_l_ew_2 = L_FA * L_FA + D_EW * D_EW
_l_ew   = math.sqrt(_l_ew_2)
_atan_e = math.atan(D_EW / L_FA)

# Joint limits (degrees) — firm safe ranges
JOINT_LIMITS = [
    (-45.0,  45.0),   # J1
    (-73.0,  90.0),   # J2
    ( 35.0, 180.0),   # J3
    (-180.0, 180.0),  # J4
    (-120.0, 120.0),  # J5
    (-720.0, 720.0),  # J6
]


# ── Helpers ────────────────────────────────────────────────────────────────────

def _euler_to_rot(a_deg: float, b_deg: float, c_deg: float) -> list[float]:
    """
    firmware EulerAngleToRotMat: R = Rz(c) @ Ry(b) @ Rx(a).
    Returns flat 9-element row-major rotation matrix.
    (Firmware uses variable names cc/sc=cos/sin(a), cb/sb=cos/sin(b),
     ca/sa=cos/sin(c) — mirrored exactly below.)
    """
    cc, sc = math.cos(math.radians(a_deg)), math.sin(math.radians(a_deg))
    cb, sb = math.cos(math.radians(b_deg)), math.sin(math.radians(b_deg))
    ca, sa = math.cos(math.radians(c_deg)), math.sin(math.radians(c_deg))
    return [
        ca*cb,            ca*sb*sc - sa*cc,  ca*sb*cc + sa*sc,
        sa*cb,            sa*sb*sc + ca*cc,  sa*sb*cc - ca*sc,
        -sb,              cb*sc,             cb*cc,
    ]


def _rot_to_euler(R: list[float]) -> tuple[float, float, float]:
    """
    firmware RotMatToEulerAngle — returns (a_deg, b_deg, c_deg).
    """
    if abs(R[6]) >= 1.0 - 1e-4:
        if R[6] < 0:
            a = 0.0
            b = math.pi / 2
            c = math.atan2(R[1], R[4])
        else:
            a = 0.0
            b = -math.pi / 2
            c = -math.atan2(R[1], R[4])
    else:
        cb = math.sqrt(R[0]*R[0] + R[3]*R[3])
        b = math.atan2(-R[6], cb)
        a = math.atan2(R[3] / cb, R[0] / cb)
        c = math.atan2(R[7] / cb, R[8] / cb)
    return math.degrees(c), math.degrees(b), math.degrees(a)


def _mat_mul_3x3_3x1(M: list[float], v: list[float]) -> list[float]:
    return [
        M[0]*v[0] + M[1]*v[1] + M[2]*v[2],
        M[3]*v[0] + M[4]*v[1] + M[5]*v[2],
        M[6]*v[0] + M[7]*v[1] + M[8]*v[2],
    ]


def _mat_mul_3x3_3x3(A: list[float], B: list[float]) -> list[float]:
    R = [0.0] * 9
    for i in range(3):
        for j in range(3):
            for k in range(3):
                R[i*3+j] += A[i*3+k] * B[k*3+j]
    return R


def _wrap(angle_rad: float) -> float:
    """Wrap angle to (−π, π] exactly as firmware does."""
    if angle_rad > math.pi:
        return angle_rad - math.pi
    if angle_rad < -math.pi:
        return angle_rad + math.pi
    return angle_rad


# ── Core solver ────────────────────────────────────────────────────────────────

def solve_ik(
    x_mm: float, y_mm: float, z_mm: float,
    a_deg: float, b_deg: float, c_deg: float,
    current_joints_deg: list[float] = None,
) -> Optional[list[float]]:
    """
    Analytical IK — exact Python port of firmware SolveIK + MoveL selection.

    Args:
        x_mm, y_mm, z_mm  : target tip position in millimetres
        a_deg, b_deg, c_deg: target wrist orientation in degrees
        current_joints_deg : current joint angles (degrees) for closest-solution
                             selection (same as firmware uses currentJoints).
                             If None, zeros are used (less stable for streaming).

    Returns:
        Best joint configuration as [J1..J6] degrees (physical angles),
        or None if no valid IK solution exists within joint limits.
    """
    if current_joints_deg is None:
        current_joints_deg = [0.0] * 6

    # ── 1. Position and rotation of target ──
    px = x_mm / 1000.0
    py = y_mm / 1000.0
    pz = z_mm / 1000.0
    R06 = _euler_to_rot(a_deg, b_deg, c_deg)

    # ── 2. Wrist centre: P_w = P06 - R06 @ L6_wrist ──
    L0_wt = _mat_mul_3x3_3x1(R06, L6_WRIST)
    P0_w  = [px - L0_wt[0], py - L0_wt[1], pz - L0_wt[2]]

    # ── 3. Solve J1 (two solutions: shoulder-front / shoulder-back) ──
    xy_dist = math.sqrt(P0_w[0]**2 + P0_w[1]**2)
    if xy_dist <= 1e-6:
        qs = [current_joints_deg[0], current_joints_deg[0]]
    else:
        q = math.atan2(P0_w[1], P0_w[0])
        qs = [q, math.atan2(-P0_w[1], -P0_w[0])]

    configs = []  # each entry: [q1..q6] in radians

    for ind_arm in range(2):
        q1 = qs[ind_arm]
        cosqs = math.cos(q1 + DH_THETA[0])
        sinqs = math.sin(q1 + DH_THETA[0])

        # R10: inverse of first joint rotation
        R10 = [
            cosqs,   sinqs, 0.0,
            0.0,     0.0,  -1.0,
            -sinqs,  cosqs, 0.0,
        ]

        P1_w   = _mat_mul_3x3_3x1(R10, P0_w)
        L1_sw  = [P1_w[i] - L1_BASE[i] for i in range(3)]
        l_sw_2 = L1_sw[0]**2 + L1_sw[1]**2
        l_sw   = math.sqrt(l_sw_2)

        # ── 4. Solve J2, J3 (two elbow solutions each) ──
        EPS = 1e-6
        if abs(_l_se + _l_ew - l_sw) <= EPS or l_sw > _l_se + _l_ew:
            # arm fully extended or unreachable
            qa_base = math.atan2(L1_sw[1], L1_sw[0])
            qa_list = [(qa_base, 0.0), (qa_base, 0.0)]
        elif abs(l_sw - abs(_l_se - _l_ew)) <= EPS or l_sw < abs(_l_se - _l_ew):
            qa_base = math.atan2(L1_sw[1], L1_sw[0])
            sign = 1.0 if ind_arm == 0 else -1.0
            qa_list = [(qa_base, sign * math.pi), (qa_base, -sign * math.pi)]
        else:
            atan_a = math.atan2(L1_sw[1], L1_sw[0])
            cos_a  = ((_l_se_2 + l_sw_2 - _l_ew_2) / (2.0 * _l_se * l_sw))
            cos_a  = max(-1.0, min(1.0, cos_a))
            acos_a = math.acos(cos_a)
            cos_e  = ((_l_se_2 + _l_ew_2 - l_sw_2) / (2.0 * _l_se * _l_ew))
            cos_e  = max(-1.0, min(1.0, cos_e))
            acos_e = math.acos(cos_e)
            atan_e = _atan_e  # elbow offset angle from module constant
            if ind_arm == 0:
                qa_list = [
                    (atan_a - acos_a + math.pi/2, atan_e - acos_e + math.pi),
                    (atan_a + acos_a + math.pi/2, atan_e + acos_e - math.pi),
                ]
            else:
                qa_list = [
                    (atan_a + acos_a + math.pi/2, atan_e + acos_e - math.pi),
                    (atan_a - acos_a + math.pi/2, atan_e - acos_e + math.pi),
                ]

        for ind_elbow, (qa0, qa1) in enumerate(qa_list):
            # ── 5. Compute R30 = R31 @ R10 ──
            cosqa0 = math.cos(qa0 + DH_THETA[1])
            sinqa0 = math.sin(qa0 + DH_THETA[1])
            cosqa1 = math.cos(qa1 + DH_THETA[2])
            sinqa1 = math.sin(qa1 + DH_THETA[2])

            R31 = [
                cosqa0*cosqa1 - sinqa0*sinqa1,  cosqa0*sinqa1 + sinqa0*cosqa1, 0.0,
                0.0,                            0.0,                           1.0,
                cosqa0*sinqa1 + sinqa0*cosqa1, -cosqa0*cosqa1 + sinqa0*sinqa1, 0.0,
            ]
            R30 = _mat_mul_3x3_3x3(R31, R10)
            R36 = _mat_mul_3x3_3x3(R30, R06)

            # ── 6. Solve J4, J5, J6 (two wrist solutions) ──
            if R36[8] >= 1.0 - 1e-6:
                qw5_list  = [0.0, 0.0]
                singular  = True
                cosqw     = 1.0
            elif R36[8] <= -1.0 + 1e-6:
                cosqw    = -1.0
                sign      = 1.0 if ind_arm == 0 else -1.0
                qw5_list  = [sign * math.pi, -sign * math.pi]
                singular  = True
            else:
                cosqw    = R36[8]
                acos_val  = math.acos(cosqw)
                if ind_arm == 0:
                    qw5_list = [acos_val, -acos_val]
                else:
                    qw5_list = [-acos_val, acos_val]
                singular  = False

            for ind_wrist, qw5 in enumerate(qw5_list):
                if singular:
                    # gimbal-lock: J4 and J6 are coupled
                    if ind_arm == 0:
                        if ind_wrist == 0:
                            qw4 = math.radians(current_joints_deg[3])
                            cw  = math.cos(qw4 + DH_THETA[3])
                            sw  = math.sin(qw4 + DH_THETA[3])
                            qw6 = math.atan2(cw*R36[3] - sw*R36[0], cw*R36[0] + sw*R36[3])
                        else:
                            qw6 = math.radians(current_joints_deg[5])
                            cw  = math.cos(qw6 + DH_THETA[5])
                            sw  = math.sin(qw6 + DH_THETA[5])
                            qw4 = math.atan2(cw*R36[3] - sw*R36[0], cw*R36[0] + sw*R36[3])
                    else:
                        if ind_wrist == 0:
                            qw6 = math.radians(current_joints_deg[5])
                            cw  = math.cos(qw6 + DH_THETA[5])
                            sw  = math.sin(qw6 + DH_THETA[5])
                            qw4 = math.atan2(cw*R36[3] - sw*R36[0], cw*R36[0] + sw*R36[3])
                        else:
                            qw4 = math.radians(current_joints_deg[3])
                            cw  = math.cos(qw4 + DH_THETA[3])
                            sw  = math.sin(qw4 + DH_THETA[3])
                            qw6 = math.atan2(cw*R36[3] - sw*R36[0], cw*R36[0] + sw*R36[3])
                else:
                    if ind_arm == 0:
                        if ind_wrist == 0:
                            qw4 = math.atan2( R36[5],  R36[2])
                            qw6 = math.atan2( R36[7], -R36[6])
                        else:
                            qw4 = math.atan2(-R36[5], -R36[2])
                            qw6 = math.atan2(-R36[7],  R36[6])
                    else:
                        if ind_wrist == 0:
                            qw4 = math.atan2(-R36[5], -R36[2])
                            qw6 = math.atan2(-R36[7],  R36[6])
                        else:
                            qw4 = math.atan2( R36[5],  R36[2])
                            qw6 = math.atan2( R36[7], -R36[6])

                # ── 7. Wrap and convert to degrees ──
                joints_rad = [q1, qa0, qa1, qw4, qw5, qw6]
                joints_deg = [math.degrees(_wrap(q)) for q in joints_rad]
                configs.append(joints_deg)

    # ── 8. Filter by joint limits ──
    valid = []
    for jd in configs:
        ok = True
        for i, (lo, hi) in enumerate(JOINT_LIMITS):
            if jd[i] < lo or jd[i] > hi:
                ok = False
                break
        if ok:
            valid.append(jd)

    if not valid:
        return None

    # ── 9. Pick solution closest to current joints (min max-joint-delta) ──
    def max_delta(jd):
        return max(abs(jd[i] - current_joints_deg[i]) for i in range(6))

    return min(valid, key=max_delta)


def solve_ik_batch(
    poses: list[tuple],                   # [(x,y,z,a,b,c), ...]
    start_joints: list[float] = None,     # starting joint config
) -> list[Optional[list[float]]]:
    """
    Solve IK for a sequence of poses, chaining each solution as the
    'current joints' reference for the next step — ensures consistent
    elbow/shoulder configuration throughout the path.
    """
    results = []
    current = start_joints or [0.0] * 6
    for pose in poses:
        sol = solve_ik(*pose, current_joints_deg=current)
        results.append(sol)
        if sol is not None:
            current = sol
    return results


# ── Self-test ──────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=== IK self-test ===\n")

    # Test round-trip: FK → IK → should recover original joints
    test_poses = [
        # (joints,  label)
        ([0, 0, 90, 0, 0, 0],       "HOME"),
        ([0, -73, 180, 0, 0, 0],    "REST"),
        ([0, -45, 140, 0, 0, 0],    "Z-HOME"),
    ]

    from ik_solver import _euler_to_rot, _mat_mul_3x3_3x1, _mat_mul_3x3_3x3

    def fk(joints_deg):
        import math
        DH = [
            [0.0,       L_BS, D_BS, -math.pi/2],
            [-math.pi/2, 0,  L_AM,  0],
            [ math.pi/2, D_EW, 0,   math.pi/2],
            [0, L_FA, 0, -math.pi/2],
            [0, 0,    0,  math.pi/2],
            [0, L_WT, 0,  0],
        ]
        def rot(theta, alpha):
            c, s   = math.cos(theta), math.sin(theta)
            ca, sa = math.cos(alpha), math.sin(alpha)
            return [c, -ca*s, sa*s, s, ca*c, -sa*c, 0, sa, ca]

        Rs = [rot(math.radians(j)+DH[i][0], DH[i][3]) for i,j in enumerate(joints_deg)]
        R02 = _mat_mul_3x3_3x3(Rs[0], Rs[1])
        R03 = _mat_mul_3x3_3x3(R02, Rs[2])
        R06 = _mat_mul_3x3_3x3(R03, _mat_mul_3x3_3x3(Rs[3], _mat_mul_3x3_3x3(Rs[4], Rs[5])))
        pos = [
            sum(Rs[0][i*3+j]*L1_BASE[j] for j in range(3)) +
            sum(R02[i*3+j]*[L_AM,0,0][j] for j in range(3)) +
            sum(R03[i*3+j]*[-D_EW,0,L_FA][j] for j in range(3)) +
            sum(R06[i*3+j]*L6_WRIST[j] for j in range(3))
            for i in range(3)
        ]
        a_deg, b_deg, c_deg = _rot_to_euler(R06)
        return [p*1000 for p in pos] + [a_deg, b_deg, c_deg]

    print(f"{'Pose':<10} {'FK pose (x,y,z,a,b,c)':^55}  IK result  max_err(°)")
    print("-"*95)
    for joints, label in test_poses:
        pose = fk(joints)
        sol  = solve_ik(*pose, current_joints_deg=joints)
        if sol:
            err = max(abs(sol[i]-joints[i]) for i in range(6))
            print(f"{label:<10}  x={pose[0]:.1f} y={pose[1]:.1f} z={pose[2]:.1f} "
                  f"a={pose[3]:.1f} b={pose[4]:.1f} c={pose[5]:.1f}  "
                  f"{'✅' if err<0.01 else '⚠️ '} err={err:.6f}°")
        else:
            print(f"{label:<10}  ❌ no solution")
