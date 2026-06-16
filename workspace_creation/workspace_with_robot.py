#!/usr/bin/env python3
"""
Reachable workspace WITH the robot drawn inside it.

Reuses the verified DH forward kinematics from ``workspace_mdh.py`` (same
``dh_param_7dof_v2.pdf`` table) to (a) Monte-Carlo sample the reachable point
cloud and (b) trace the arm's kinematic skeleton at a representative pose, then
overlays the two so the manipulator is shown sitting inside its own workspace
envelope.

    python3 workspace_with_robot.py                 # default pose + 200k cloud
    python3 workspace_with_robot.py -n 400000
    python3 workspace_with_robot.py --pose 0 -0.7 0 1.4 0 0 1.0

Outputs ``workspace_with_robot_3d.{png,pdf}`` and
``workspace_with_robot_projections.{png,pdf}``.  Needs only numpy + matplotlib.
"""
import argparse
import numpy as np

# Reuse the exact DH kinematics + sampling from the workspace generator.
from workspace_mdh import (
    DH_FRAMES, dh_matrices, fk_batch, sample_workspace, Q_MIN, Q_MAX,
    L1, L2, L3, L4,
)

# A representative reaching pose, expressed in THIS DH model (which maps joints
# differently than the URDF — joint_2 is a roll that doesn't bend the arm, while
# joint_3/joint_5 are the bending joints). Base rotated (j1), elbow out (j3) and
# wrist bent (j5) so the arm reaches out into the mid-workspace and reads clearly
# as a manipulator rather than the straight-up near-singular home stretch.
DEFAULT_POSE = np.array([0.6, 0.0, 1.0, 0.0, 0.7, 0.0, 0.0])

# Which DH frame origins carry a movable joint (0-based into DH_FRAMES) — used
# to mark the joint centres on the skeleton.
JOINT_FRAME_ROWS = [i for i, f in enumerate(DH_FRAMES) if f[3] is not None]


def fk_frames(q, convention="std"):
    """Origins (M+1, 3) of the base frame followed by every DH frame, for a
    single joint vector ``q``.  Connecting them in order traces the arm."""
    q = np.asarray(q, float)
    T = np.eye(4)
    origins = [T[:3, 3].copy()]
    for (a, alpha, d, jidx, toff) in DH_FRAMES:
        theta = (q[jidx] + toff) if jidx is not None else toff
        T = T @ dh_matrices(a, alpha, d, np.array([theta]), convention)[0]
        origins.append(T[:3, 3].copy())
    return np.array(origins)


def _dedupe_polyline(pts, tol=1e-6):
    """Drop consecutive coincident points (the d=0 wrist-twist frames sit on
    top of each other) so the link polyline + joint markers stay clean."""
    out = [pts[0]]
    for p in pts[1:]:
        if np.linalg.norm(p - out[-1]) > tol:
            out.append(p)
    return np.array(out)


def make_figure(P, frames, convention, out_prefix, max_plot=60000):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    rng = np.random.default_rng(0)
    Pp = P[rng.choice(P.shape[0], min(max_plot, P.shape[0]), replace=False)]
    x, y, z = Pp[:, 0], Pp[:, 1], Pp[:, 2]

    link = _dedupe_polyline(frames)
    ee = frames[-1]
    joints = frames[[r + 1 for r in JOINT_FRAME_ROWS]]   # +1: base is row 0

    ARM = "#d8232a"      # arm links (red)
    JOINT = "#1b1b1b"    # joint markers
    EE = "#10a050"       # end-effector

    def draw_arm_3d(ax):
        ax.plot(link[:, 0], link[:, 1], link[:, 2], "-", color=ARM, lw=4,
                solid_capstyle="round", zorder=6)
        ax.scatter(joints[:, 0], joints[:, 1], joints[:, 2], c=JOINT, s=34,
                   depthshade=False, zorder=7)
        ax.scatter([0], [0], [0], c=JOINT, s=130, marker="s",
                   depthshade=False, zorder=7, label="base")
        ax.scatter([ee[0]], [ee[1]], [ee[2]], c=EE, s=70, marker="*",
                   depthshade=False, zorder=8, label="end-effector")

    # ── 3D: cloud + arm ────────────────────────────────────────────────────
    fig = plt.figure(figsize=(8.2, 7.2))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(x, y, z, c=z, cmap="viridis", s=2, alpha=0.16, linewidths=0)
    draw_arm_3d(ax)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    ax.set_title("7-DOF arm inside its reachable workspace\n"
                 f"(Monte-Carlo DH FK, {P.shape[0]:,} samples)")
    ax.legend(loc="upper left")
    try:
        ax.set_box_aspect((np.ptp(x), np.ptp(y), np.ptp(z)))
    except Exception:
        pass
    ax.view_init(elev=22, azim=-60)
    fig.tight_layout()
    fig.savefig(f"{out_prefix}_3d.png", dpi=160)
    fig.savefig(f"{out_prefix}_3d.pdf")
    plt.close(fig)

    # ── projections: cloud + arm ───────────────────────────────────────────
    r_cloud = np.hypot(x, y)
    r_link = np.hypot(link[:, 0], link[:, 1])
    r_joint = np.hypot(joints[:, 0], joints[:, 1])
    fig, axs = plt.subplots(1, 3, figsize=(15, 5.2))

    def arm_2d(ax, ax_pts, a0, a1):
        ax.plot(ax_pts[:, a0], ax_pts[:, a1], "-", color=ARM, lw=3.5,
                solid_capstyle="round", zorder=6)
        ax.scatter(joints[:, a0], joints[:, a1], c=JOINT, s=26, zorder=7)
        ax.scatter([ee[a0]], [ee[a1]], c=EE, s=80, marker="*", zorder=8)

    axs[0].scatter(x, y, s=1, alpha=0.18, c=z, cmap="viridis", linewidths=0)
    arm_2d(axs[0], link, 0, 1)
    axs[0].plot(0, 0, "s", color=JOINT, ms=8)
    axs[0].set_title("Top view (X–Y)"); axs[0].set_xlabel("x [m]"); axs[0].set_ylabel("y [m]")

    axs[1].scatter(x, z, s=1, alpha=0.18, c=z, cmap="viridis", linewidths=0)
    arm_2d(axs[1], link, 0, 2)
    axs[1].plot(0, 0, "s", color=JOINT, ms=8)
    axs[1].set_title("Front view (X–Z)"); axs[1].set_xlabel("x [m]"); axs[1].set_ylabel("z [m]")

    axs[2].scatter(r_cloud, z, s=1, alpha=0.18, c=z, cmap="viridis", linewidths=0)
    axs[2].plot(r_link, link[:, 2], "-", color=ARM, lw=3.5, solid_capstyle="round", zorder=6)
    axs[2].scatter(r_joint, joints[:, 2], c=JOINT, s=26, zorder=7)
    axs[2].scatter([np.hypot(ee[0], ee[1])], [ee[2]], c=EE, s=80, marker="*", zorder=8)
    axs[2].plot(0, 0, "s", color=JOINT, ms=8)
    axs[2].set_title("Side profile (radius r vs Z)")
    axs[2].set_xlabel("r = √(x²+y²) [m]"); axs[2].set_ylabel("z [m]")

    for a in axs:
        a.set_aspect("equal", "box"); a.grid(True, alpha=0.3)
    fig.suptitle(f"7-DOF arm in its reachable workspace — orthographic projections ({convention} DH)")
    fig.tight_layout()
    fig.savefig(f"{out_prefix}_projections.png", dpi=160)
    fig.savefig(f"{out_prefix}_projections.pdf")
    plt.close(fig)
    print(f"wrote {out_prefix}_3d.png/.pdf and {out_prefix}_projections.png/.pdf")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-n", "--samples", type=int, default=200_000)
    ap.add_argument("--convention", choices=["std", "mdh"], default="std")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--pose", type=float, nargs=7, default=None,
                    metavar="q",
                    help="7 joint angles [rad] for the drawn arm "
                         "(default = elbow-up go_to_start pose)")
    ap.add_argument("--out", default="workspace_with_robot")
    args = ap.parse_args()

    pose = np.array(args.pose) if args.pose is not None else DEFAULT_POSE
    _, P = sample_workspace(args.samples, args.convention, args.seed, Q_MIN, Q_MAX)
    frames = fk_frames(pose, args.convention)
    ee = frames[-1]
    print(f"drawn pose      : {pose.round(3).tolist()}")
    print(f"EE position     : {ee.round(4).tolist()}  (|p|={np.linalg.norm(ee):.4f} m)")
    print(f"cloud z-range   : [{P[:,2].min():+.3f}, {P[:,2].max():+.3f}] m")
    make_figure(P, frames, args.convention, args.out)


if __name__ == "__main__":
    main()
