#!/usr/bin/env python3
"""
jog_envelope.py — reachable-workspace envelope for the *Cartesian-jog* IK.

Unlike workspace_mdh.py (analytical DH, FULL URDF limits — a thesis artifact),
this samples the **URDF chain** under the **jog soft-limits** actually enforced
by the pendant's jog IK node (ik_7dof_v3 in pendant_backend.launch.py:
elbow-up joint_4 in [0.05, 1.6], etc.). That makes it an *honest* model of what
the position-only jog can reach, suitable for a runtime workspace clamp in the
pendant's ros_bridge.cartesian_jog_xyz.

The jog arm is axisymmetric (continuous joint_1 -> full 360 deg azimuth), so the
3-D reachable set collapses to a 2-D region in the (r, z) half-plane, r = hypot(x, y).
We emit the OUTER boundary r_max(z) plus the z extent — enough to clamp/classify
a jog target. Run with the workspace sourced:

    source /opt/ros/humble/setup.bash && source install/setup.bash
    python3 workspace_creation/jog_envelope.py --urdf /tmp/arm_bot_fk.urdf

Prints a Python literal (paste into ros_bridge) and writes jog_workspace_envelope.json.
"""
import argparse
import json

import numpy as np

from arm_bot.ik_lib import UrdfChain


# Jog soft-limits — MUST match pendant_backend.launch.py ik_7dof_v3
# soft_q_min / soft_q_max (elbow-up filter). Keep in sync if those change.
SOFT_Q_MIN = np.array([-3.14, -1.6, -3.14, -1.6, -1.6, -0.48, -1.6])
SOFT_Q_MAX = np.array([ 3.14,  1.6,  3.14,  1.6,  1.6,  0.26,  1.6])


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default="/tmp/arm_bot_fk.urdf",
                    help="URDF xml file (xacro the arm first)")
    ap.add_argument("--base", default="base_link")
    ap.add_argument("--tip", default="ee")
    ap.add_argument("-n", "--samples", type=int, default=400_000)
    ap.add_argument("--zbins", type=int, default=28)
    ap.add_argument("--min-per-bin", type=int, default=40,
                    help="drop z-bins with fewer samples (sparse extremes)")
    ap.add_argument("--out", default="workspace_creation/jog_workspace_envelope.json")
    args = ap.parse_args()

    with open(args.urdf) as f:
        urdf_xml = f.read()
    chain = UrdfChain(urdf_xml, args.base, args.tip)
    assert chain.n == 7, f"expected 7 DOF, got {chain.n}"

    # Intersect URDF hard limits with the jog soft box -> the box the jog samples.
    lo = np.maximum(chain.q_min, SOFT_Q_MIN)
    hi = np.minimum(chain.q_max, SOFT_Q_MAX)
    print(f"[jog_envelope] sampling box lo={np.round(lo,3)} hi={np.round(hi,3)}")

    rng = np.random.default_rng(0)
    N = args.samples
    Q = lo + (hi - lo) * rng.random((N, 7))

    # FK every sample -> (r, z). (Vectorising FK isn't worth it; chain.fk is
    # a tiny 4x4 chain — 400k calls run in a few seconds.)
    r = np.empty(N)
    z = np.empty(N)
    for i in range(N):
        _, T = chain.fk(Q[i])
        x, y, zz = T[:3, 3]
        r[i] = np.hypot(x, y)
        z[i] = zz

    z_min, z_max = float(z.min()), float(z.max())
    r_max_overall = float(r.max())
    print(f"[jog_envelope] z in [{z_min:.3f}, {z_max:.3f}]  r_max={r_max_overall:.3f}")

    # Outer boundary r_max(z): bin z, take the max r per bin.
    edges = np.linspace(z_min, z_max, args.zbins + 1)
    centers = 0.5 * (edges[:-1] + edges[1:])
    idx = np.clip(np.digitize(z, edges) - 1, 0, args.zbins - 1)
    z_knots, r_knots = [], []
    for b in range(args.zbins):
        m = idx == b
        if int(m.sum()) < args.min_per_bin:
            continue
        z_knots.append(round(float(centers[b]), 4))
        r_knots.append(round(float(r[m].max()), 4))

    env = {
        "frame": args.base,
        "tip": args.tip,
        "convention": "urdf_fk_jog_softlimits",
        "n_samples": N,
        "soft_q_min": SOFT_Q_MIN.tolist(),
        "soft_q_max": SOFT_Q_MAX.tolist(),
        "z_min_m": round(z_min, 4),
        "z_max_m": round(z_max, 4),
        "r_max_overall_m": round(r_max_overall, 4),
        "z_knots_m": z_knots,
        "r_max_knots_m": r_knots,
    }
    with open(args.out, "w") as f:
        json.dump(env, f, indent=2)
    print(f"[jog_envelope] wrote {args.out}")

    # Python literal for ros_bridge (paste-ready).
    print("\n# ---- paste into ros_bridge.py ----")
    print(f"_WS_Z_MIN = {round(z_min, 4)}")
    print(f"_WS_Z_MAX = {round(z_max, 4)}")
    print(f"_WS_Z_KNOTS = {z_knots}")
    print(f"_WS_R_MAX   = {r_knots}")


if __name__ == "__main__":
    main()
