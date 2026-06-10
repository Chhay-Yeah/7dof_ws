#!/usr/bin/env python3
"""
Monte-Carlo reachable-workspace generator for the 7-DOF arm, built from the
*numerical DH parameters* in ``dh_param_7dof_v2.pdf`` (NOT the URDF).

Method
------
1. Forward kinematics is assembled directly from the 11-frame DH table below.
   Seven of the eleven frames carry a joint variable (theta_1..theta_7); the
   other four are fixed structural frames that realise the +/-pi/2 wrist twists.
2. ``N`` joint configurations are drawn uniformly at random inside the joint
   limits (Monte-Carlo sampling of the joint space).
3. FK maps every sample to an end-effector position; the resulting point cloud
   *is* a sampled estimate of the reachable workspace. Bounds, horizontal reach
   and an (approximate) volume are reported, and the cloud is plotted.

Convention
----------
The PDF titles the table "Conventional DH Parameter" and shows it verified
against the CAD model, so the DEFAULT here is the standard (Spong/distal)
convention::

    A_i = Rotz(theta_i) . Transz(d_i) . Transx(a_i) . Rotx(alpha_i)

Pass ``--convention mdh`` for the modified (Craig/proximal) form::

    A_i = Rotx(alpha_i) . Transx(a_i) . Rotz(theta_i) . Transz(d_i)

Every a_i in this table is 0, so the two conventions still differ only through
the ordering of the alpha twists -- both are provided so the result can be
cross-checked either way.

Run
---
    python3 workspace_mdh.py                 # 200k samples, std DH, writes plots
    python3 workspace_mdh.py -n 1000000      # denser cloud
    python3 workspace_mdh.py --convention mdh
Needs only numpy + matplotlib (scipy optional, for convex-hull volume).
"""
import argparse
import json
import numpy as np

# --- link lengths (mm -> m), from dh_param_7dof_v2.pdf -----------------------
L1 = 0.11875   # 118.75 mm
L2 = 0.228     # 228 mm
L3 = 0.22712   # 227.12 mm
L4 = 0.050     # 50 mm

PI2 = np.pi / 2.0

# --- the 11-frame DH table ---------------------------------------------------
# Each row: (a, alpha, d, joint_index_or_None, theta_offset)
#   joint_index in 0..6 -> theta column = q[idx] + theta_offset
#   joint_index None    -> fixed frame, theta column = theta_offset
DH_FRAMES = [
    (0.0,  0.0,   L1,   0,    0.0),     # frame 1  theta_1
    (0.0, -PI2,   0.0,  1,    0.0),     # frame 2  theta_2
    (0.0,  PI2,   0.0,  2,    0.0),     # frame 3  theta_3
    (0.0,  PI2,   L2,   3,    0.0),     # frame 4  theta_4
    (0.0, -PI2,   0.0,  4,    0.0),     # frame 5  theta_5
    (0.0, -PI2,   L3,   5,    0.0),     # frame 6  theta_6
    (0.0,  PI2,   0.0,  None, 0.0),     # frame 7  fixed
    (0.0,  0.0,   0.0,  None, -PI2),    # frame 8  fixed  theta = -pi/2
    (0.0, -PI2,   0.0,  6,    0.0),     # frame 9  theta_7
    (0.0,  PI2,   0.0,  None, 0.0),     # frame 10 fixed
    (0.0,  0.0,   L4,   None, 0.0),     # frame 11 fixed  (tool length)
]

# --- joint limits (rad), from the URDF -------------------------------------
#   joint_1/3/5 are 'continuous' in the URDF -> sampled over [-pi, pi]
#   joint_2/4/7 'revolute' [-1.6, 1.6];  joint_6 restricted [-0.48, 0.26]
PI = np.pi
Q_MIN = np.array([-PI, -1.6, -PI, -1.6, -PI, -0.48, -1.6])
Q_MAX = np.array([ PI,  1.6,  PI,  1.6,  PI,  0.26,  1.6])
JOINT_NAMES = [f"joint_{i+1}" for i in range(7)]


def dh_matrices(a, alpha, d, theta, convention):
    """Return a batched (N,4,4) DH transform for arrays of theta (and scalars
    a, alpha, d). ``convention`` is 'std' or 'mdh'."""
    theta = np.atleast_1d(theta).astype(float)
    n = theta.shape[0]
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    T = np.zeros((n, 4, 4))
    if convention == "std":
        # Rotz(theta) Transz(d) Transx(a) Rotx(alpha)
        T[:, 0, 0] = ct
        T[:, 0, 1] = -st * ca
        T[:, 0, 2] = st * sa
        T[:, 0, 3] = a * ct
        T[:, 1, 0] = st
        T[:, 1, 1] = ct * ca
        T[:, 1, 2] = -ct * sa
        T[:, 1, 3] = a * st
        T[:, 2, 1] = sa
        T[:, 2, 2] = ca
        T[:, 2, 3] = d
        T[:, 3, 3] = 1.0
    elif convention == "mdh":
        # Rotx(alpha) Transx(a) Rotz(theta) Transz(d)
        T[:, 0, 0] = ct
        T[:, 0, 1] = -st
        T[:, 0, 3] = a
        T[:, 1, 0] = st * ca
        T[:, 1, 1] = ct * ca
        T[:, 1, 2] = -sa
        T[:, 1, 3] = -d * sa
        T[:, 2, 0] = st * sa
        T[:, 2, 1] = ct * sa
        T[:, 2, 2] = ca
        T[:, 2, 3] = d * ca
        T[:, 3, 3] = 1.0
    else:
        raise ValueError(f"unknown convention {convention!r}")
    return T


def fk_batch(Q, convention="std"):
    """Forward kinematics for a batch of joint vectors.

    Q : (N,7) array of joint angles
    returns (N,3) end-effector positions in the base frame (metres).
    """
    Q = np.atleast_2d(Q).astype(float)
    n = Q.shape[0]
    T = np.broadcast_to(np.eye(4), (n, 4, 4)).copy()
    for (a, alpha, d, jidx, toff) in DH_FRAMES:
        theta = (Q[:, jidx] + toff) if jidx is not None else np.full(n, toff)
        T = T @ dh_matrices(a, alpha, d, theta, convention)
    return T[:, :3, 3]


def sample_workspace(n, convention, seed, qmin, qmax):
    rng = np.random.default_rng(seed)
    Q = qmin + (qmax - qmin) * rng.random((n, 7))
    P = fk_batch(Q, convention)
    return Q, P


def convex_hull_volume(P):
    try:
        from scipy.spatial import ConvexHull
    except Exception:
        return None
    try:
        return float(ConvexHull(P).volume)
    except Exception:
        return None


def voxel_volume(P, voxel=0.02):
    """Occupied-voxel volume estimate (better for a non-convex shell)."""
    keys = np.floor(P / voxel).astype(np.int64)
    occupied = len(np.unique(keys, axis=0))
    return occupied * (voxel ** 3), occupied, voxel


def report(P, convention, hull_v, vox):
    vol_vox, n_vox, vsz = vox
    r = np.hypot(P[:, 0], P[:, 1])
    reach = np.linalg.norm(P, axis=1)
    m = {
        "convention": convention,
        "n_samples": int(P.shape[0]),
        "x_m": [float(P[:, 0].min()), float(P[:, 0].max())],
        "y_m": [float(P[:, 1].min()), float(P[:, 1].max())],
        "z_m": [float(P[:, 2].min()), float(P[:, 2].max())],
        "horizontal_radius_m": [float(r.min()), float(r.max())],
        "reach_from_base_m": [float(reach.min()), float(reach.max())],
        "convex_hull_volume_m3": hull_v,
        "voxel_volume_m3": vol_vox,
        "voxel_count": int(n_vox),
        "voxel_size_m": vsz,
        "link_lengths_m": {"l1": L1, "l2": L2, "l3": L3, "l4": L4},
        "joint_limits_rad": {JOINT_NAMES[i]: [float(Q_MIN[i]), float(Q_MAX[i])]
                             for i in range(7)},
    }
    print("\n=== Reachable workspace (Monte-Carlo, DH FK) ===")
    print(f"convention      : {convention}")
    print(f"samples         : {m['n_samples']:,}")
    print(f"x range         : [{m['x_m'][0]:+.3f}, {m['x_m'][1]:+.3f}] m")
    print(f"y range         : [{m['y_m'][0]:+.3f}, {m['y_m'][1]:+.3f}] m")
    print(f"z range         : [{m['z_m'][0]:+.3f}, {m['z_m'][1]:+.3f}] m")
    print(f"horiz radius r  : [{m['horizontal_radius_m'][0]:.3f}, "
          f"{m['horizontal_radius_m'][1]:.3f}] m")
    print(f"reach from base : [{m['reach_from_base_m'][0]:.3f}, "
          f"{m['reach_from_base_m'][1]:.3f}] m")
    if hull_v is not None:
        print(f"convex-hull vol : {hull_v:.4f} m^3")
    print(f"voxel vol (~{vsz*1000:.0f}mm): {vol_vox:.4f} m^3  ({n_vox} voxels)")
    return m


def make_plots(P, convention, out_prefix, max_plot=40000):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    # subsample for plotting speed
    if P.shape[0] > max_plot:
        idx = np.random.default_rng(0).choice(P.shape[0], max_plot, replace=False)
        Pp = P[idx]
    else:
        Pp = P
    x, y, z = Pp[:, 0], Pp[:, 1], Pp[:, 2]
    c = z  # colour by height

    # 3D scatter
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(x, y, z, c=c, cmap="viridis", s=1, alpha=0.25, linewidths=0)
    ax.scatter([0], [0], [0], c="red", s=40, marker="^", label="base")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    ax.set_title(f"7-DOF reachable workspace  (Monte-Carlo, {convention} DH, "
                 f"{P.shape[0]:,} samples)")
    ax.legend(loc="upper right")
    try:
        ax.set_box_aspect((np.ptp(x), np.ptp(y), np.ptp(z)))
    except Exception:
        pass
    fig.tight_layout()
    fig.savefig(f"{out_prefix}_3d.png", dpi=150)
    fig.savefig(f"{out_prefix}_3d.pdf")
    plt.close(fig)

    # three orthographic projections
    r = np.hypot(x, y)
    fig, axs = plt.subplots(1, 3, figsize=(15, 5))
    axs[0].scatter(x, y, s=1, alpha=0.2, c=c, cmap="viridis", linewidths=0)
    axs[0].plot(0, 0, "r^"); axs[0].set_title("Top view  (X-Y)")
    axs[0].set_xlabel("x [m]"); axs[0].set_ylabel("y [m]")
    axs[1].scatter(x, z, s=1, alpha=0.2, c=c, cmap="viridis", linewidths=0)
    axs[1].plot(0, 0, "r^"); axs[1].set_title("Front view  (X-Z)")
    axs[1].set_xlabel("x [m]"); axs[1].set_ylabel("z [m]")
    axs[2].scatter(r, z, s=1, alpha=0.2, c=c, cmap="viridis", linewidths=0)
    axs[2].plot(0, 0, "r^"); axs[2].set_title("Side profile  (radius r vs Z)")
    axs[2].set_xlabel("r = sqrt(x^2+y^2) [m]"); axs[2].set_ylabel("z [m]")
    for a in axs:
        a.set_aspect("equal", "box"); a.grid(True, alpha=0.3)
    fig.suptitle(f"7-DOF reachable workspace projections ({convention} DH)")
    fig.tight_layout()
    fig.savefig(f"{out_prefix}_projections.png", dpi=150)
    fig.savefig(f"{out_prefix}_projections.pdf")
    plt.close(fig)
    print(f"wrote {out_prefix}_3d.png/.pdf and {out_prefix}_projections.png/.pdf")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-n", "--samples", type=int, default=200_000,
                    help="number of Monte-Carlo joint samples (default 200000)")
    ap.add_argument("--convention", choices=["std", "mdh"], default="std",
                    help="DH convention (default std = the verified table)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--voxel", type=float, default=0.02,
                    help="voxel size [m] for the volume estimate (default 0.02)")
    ap.add_argument("--out", default="workspace",
                    help="output file prefix (default 'workspace')")
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("--save-cloud", action="store_true",
                    help="also save the full point cloud as .npy")
    args = ap.parse_args()

    # sanity: print the home (all-zeros) EE pose
    home = fk_batch(np.zeros((1, 7)), args.convention)[0]
    print(f"home (q=0) EE position [{args.convention}] = "
          f"{home.round(4)}  (|p|={np.linalg.norm(home):.4f} m)")

    Q, P = sample_workspace(args.samples, args.convention, args.seed,
                            Q_MIN, Q_MAX)
    hull_v = convex_hull_volume(P)
    vox = voxel_volume(P, args.voxel)
    m = report(P, args.convention, hull_v, vox)

    with open(f"{args.out}_metrics.json", "w") as f:
        json.dump(m, f, indent=2)
    print(f"wrote {args.out}_metrics.json")

    # always save a subsampled CSV so the cloud is inspectable without numpy
    sub = P if P.shape[0] <= 50000 else P[np.random.default_rng(0).choice(
        P.shape[0], 50000, replace=False)]
    np.savetxt(f"{args.out}_cloud.csv", sub, delimiter=",",
               header="x_m,y_m,z_m", comments="")
    print(f"wrote {args.out}_cloud.csv  ({sub.shape[0]} points)")
    if args.save_cloud:
        np.save(f"{args.out}_cloud.npy", P)
        print(f"wrote {args.out}_cloud.npy  ({P.shape[0]} points)")

    if not args.no_plots:
        make_plots(P, args.convention, args.out)


if __name__ == "__main__":
    main()
