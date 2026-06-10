#!/usr/bin/env python3
"""
make_thesis_figures.py  —  emits Figures 4.1, 4.3, 4.4, 4.5 from ONE rosbag.

All four figures come from a single recording of a drawing run:

    ros2 bag record -o draw_run /joint_states /cartesian_path /robot_description
    # (start it BEFORE clicking Send so it catches the one-shot /cartesian_path)

Then:

    python3 src/arm_bot/analysis/make_thesis_figures.py draw_run

Writes (PNG @300dpi + PDF for LaTeX) into --outdir (default: cwd):
    figure_4_1.{png,pdf}   commanded vs executed path overlay (drawing plane)
    figure_4_3.{png,pdf}   joint position trajectories  (7 lines vs time)
    figure_4_4.{png,pdf}   joint velocity profiles       (7 lines vs time)
    figure_4_5.{png,pdf}   joint_6 angle vs its limits   (1 line + limit band)

Needs only a stock ROS install on the path (/opt/ros) — the FK chain is read
from the URDF in the bag, not from the arm_bot overlay. Time axis is zeroed at
the trajectory dispatch (the /cartesian_path stamp), so t=0 ≈ motion start.
"""
import argparse
import importlib.util
import os
import sys

import numpy as np
import matplotlib


# ── reuse the shared helpers from the sibling 4.1 script ─────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    'pcve', os.path.join(_HERE, 'plot_commanded_vs_executed.py'))
P = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(P)

JOINT_NAMES = [f'joint_{i}' for i in range(1, 8)]
COLORS = matplotlib.colormaps['tab10'].colors


# ── data loading ─────────────────────────────────────────────────────────────

def load_series(joints, names):
    """Returns (t[N], pos[N,7], vel[N,7]) sorted by time. Indexes by joint NAME
    every message (the bag's /joint_states name order is not sorted, and can in
    principle differ per message)."""
    idx_cache = {}
    t, pos, vel = [], [], []
    for ts, msg in joints:
        key = tuple(msg.name)
        ii = idx_cache.get(key)
        if ii is None:
            ii = [msg.name.index(n) for n in names]
            idx_cache[key] = ii
        t.append(ts)
        pos.append([msg.position[i] for i in ii])
        if len(msg.velocity) == len(msg.name):
            vel.append([msg.velocity[i] for i in ii])
        else:
            vel.append([np.nan] * len(names))
    t = np.array(t); pos = np.array(pos); vel = np.array(vel)
    order = np.argsort(t)
    return t[order], pos[order], vel[order]


def joint_limit(urdf_xml, name):
    from urdf_parser_py.urdf import URDF
    robot = URDF.from_xml_string(urdf_xml)
    for j in robot.joints:
        if j.name == name and j.limit is not None:
            return float(j.limit.lower), float(j.limit.upper)
    return None


# ── figure helpers ───────────────────────────────────────────────────────────

def save(fig, outdir, stem):
    import matplotlib.pyplot as plt
    png = os.path.join(outdir, stem + '.png')
    pdf = os.path.join(outdir, stem + '.pdf')
    fig.tight_layout()
    fig.savefig(png, dpi=300)
    fig.savefig(pdf)
    print(f'wrote {png} + {pdf}')
    plt.close(fig)


def fig_joint_positions(trel, pos, outdir, show):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7.5, 4.5))
    for k, name in enumerate(JOINT_NAMES):
        ax.plot(trel, pos[:, k], lw=1.3, color=COLORS[k], label=name)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('joint angle (rad)')
    ax.set_title('Figure 4.3 — Joint position trajectories')
    ax.grid(True, ls=':', alpha=0.5)
    ax.legend(ncol=4, fontsize=8, loc='upper right', framealpha=0.9)
    ax.set_xlim(trel.min(), trel.max())
    save(fig, outdir, 'figure_4_3')
    if show:
        plt.show()


def fig_joint_velocities(trel, vel, pos, outdir, show, source):
    import matplotlib.pyplot as plt
    label_src = 'published'
    if source == 'diff' or not np.isfinite(vel).any():
        # central-difference of positions; robust if velocity wasn't published
        v = np.gradient(pos, trel, axis=0)
        label_src = 'numeric diff'
    else:
        v = vel
    fig, ax = plt.subplots(figsize=(7.5, 4.5))
    for k, name in enumerate(JOINT_NAMES):
        ax.plot(trel, v[:, k], lw=1.3, color=COLORS[k], label=name)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('joint velocity (rad/s)')
    ax.set_title(f'Figure 4.4 — Joint velocity profiles ({label_src})')
    ax.grid(True, ls=':', alpha=0.5)
    ax.legend(ncol=4, fontsize=8, loc='upper right', framealpha=0.9)
    ax.set_xlim(trel.min(), trel.max())
    save(fig, outdir, 'figure_4_4')
    if show:
        plt.show()


def fig_joint6_limits(trel, pos, limits, outdir, show):
    import matplotlib.pyplot as plt
    j6 = pos[:, 5]                       # joint_6 is index 5
    lo, hi = limits
    fig, ax = plt.subplots(figsize=(7.5, 4.5))
    # shaded allowed band + dashed limit lines
    ax.axhspan(lo, hi, color='#2ca02c', alpha=0.10, zorder=0,
               label='allowed range')
    ax.axhline(lo, color='#d62728', ls='--', lw=1.3, label=f'lower limit ({lo:+.3f})')
    ax.axhline(hi, color='#d62728', ls='--', lw=1.3, label=f'upper limit ({hi:+.3f})')
    ax.plot(trel, j6, lw=1.6, color='#1f77b4', label='joint_6', zorder=3)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('joint_6 angle (rad)')
    ax.set_title('Figure 4.5 — Joint 6 position vs limits')
    ax.grid(True, ls=':', alpha=0.5)
    ax.set_xlim(trel.min(), trel.max())
    # headroom so the band reads clearly
    span = hi - lo
    ax.set_ylim(lo - 0.35 * span, hi + 0.35 * span)
    ax.legend(fontsize=8, loc='upper right', framealpha=0.9)
    margin_lo = float(np.min(j6) - lo)
    margin_hi = float(hi - np.max(j6))
    ax.text(0.02, 0.04,
            f'range used: [{np.min(j6):+.3f}, {np.max(j6):+.3f}] rad\n'
            f'closest approach to a limit: {min(margin_lo, margin_hi):.3f} rad',
            transform=ax.transAxes, va='bottom', ha='left', fontsize=9,
            bbox=dict(boxstyle='round', fc='white', ec='0.7', alpha=0.9))
    save(fig, outdir, 'figure_4_5')
    if show:
        plt.show()
    return float(np.min(j6)), float(np.max(j6)), margin_lo, margin_hi


# ── Figure 4.1 (commanded vs executed) via the shared helpers ────────────────

def fig_path_overlay(args, urdf, cart_msgs, joint_msgs, chain, outdir, t0_traj):
    if not cart_msgs:
        print('skip Figure 4.1 — no /cartesian_path in bag')
        return None
    begin = np.array([float(v) for v in args.begin_draw_joints.split(',')])
    pen_axis = np.array([float(v) for v in args.pen_axis_local.split(',')])
    pf = P.PaperFrame(chain, begin, args.pen_offset_mm / 1000.0, pen_axis,
                      args.paper_rotation_deg, args.paper_mirror_x)
    pa = cart_msgs[-1]
    cmd = np.array([[ps.position.x * 1000.0, ps.position.y * 1000.0,
                     ps.position.z * 1000.0] for ps in pa.poses])
    cmd_x, cmd_y, cmd_z = cmd[:, 0], cmd[:, 1], cmd[:, 2]
    draw_start = t0_traj + args.move_to_begin + args.dwell + args.settle + args.approach
    ex, ey, ez = [], [], []
    for ts, msg in joint_msgs:
        if ts < draw_start:
            continue
        q = P.joint_q(msg, chain.joint_names)
        T = chain.fk(q)
        x, y, z = pf.exec_to_paper_mm(T[:3, 3], T[:3, :3])
        ex.append(x); ey.append(y); ez.append(z)
    ex, ey, ez = np.array(ex), np.array(ey), np.array(ez)
    up = P.auto_pen_up_thresh(cmd_z)
    cx, cy = P.break_on_pen_up(cmd_x, cmd_y, cmd_z, up)
    fx, fy = P.break_on_pen_up(ex, ey, ez, up)
    rms, mx, _ = P.tracking_error(fx, fy, cx, cy)
    P.make_figure(cx, cy, fx, fy, os.path.join(outdir, 'figure_4_1.png'),
                  'paper', rms, mx, show=False)
    return rms, mx


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description='Emit thesis Figures 4.1/4.3/4.4/4.5 from one rosbag.')
    ap.add_argument('bag')
    ap.add_argument('--outdir', default='.')
    ap.add_argument('--urdf', help='URDF file if /robot_description not in the bag')
    ap.add_argument('--base-link', default='base_link')
    ap.add_argument('--tip-link', default='ee')
    ap.add_argument('--vel-source', choices=['published', 'diff'], default='published')
    ap.add_argument('--full', action='store_true',
                    help='plot the whole bag, not just from trajectory dispatch onward')
    ap.add_argument('--no-trim', action='store_true',
                    help='keep the trailing static hold instead of trimming it')
    ap.add_argument('--static-eps', type=float, default=0.01,
                    help='|velocity| (rad/s) below which the arm counts as stopped (tail trim)')
    ap.add_argument('--tail-pad', type=float, default=1.0,
                    help='seconds of settle kept after the last motion')
    ap.add_argument('--show', action='store_true')
    # Figure 4.1 paper-frame params (defaults = pendant_backend.launch.py)
    ap.add_argument('--begin-draw-joints', default='0.0,-0.7,0.0,1.4,0.01,0.0,1.0')
    ap.add_argument('--pen-offset-mm', type=float, default=100.0)
    ap.add_argument('--pen-axis-local', default='1,0,0')
    ap.add_argument('--paper-rotation-deg', type=float, default=270.0)
    ap.add_argument('--paper-mirror-x', action='store_true')
    ap.add_argument('--move-to-begin', type=float, default=4.0)
    ap.add_argument('--dwell', type=float, default=3.0)
    ap.add_argument('--settle', type=float, default=0.5)
    ap.add_argument('--approach', type=float, default=1.0)
    args = ap.parse_args()

    if not args.show:
        matplotlib.use('Agg')
    os.makedirs(args.outdir, exist_ok=True)

    print(f'reading bag: {args.bag}')
    urdf, cart_msgs, joint_msgs = P.read_bag(args.bag)
    if urdf is None and args.urdf:
        urdf = open(args.urdf).read()
    if urdf is None:
        sys.exit('ERROR: no /robot_description in bag and no --urdf given.')
    if not joint_msgs:
        sys.exit('ERROR: no /joint_states in bag.')
    chain = P.UrdfChain(urdf, args.base_link, args.tip_link)
    print(f'URDF chain: {chain.n} DoF — {chain.joint_names}')

    # time origin = trajectory dispatch (one-shot /cartesian_path stamp)
    if cart_msgs:
        pa = cart_msgs[-1]
        t0 = pa.header.stamp.sec + pa.header.stamp.nanosec * 1e-9
    else:
        t0 = min(ts for ts, _ in joint_msgs)

    t, pos, vel = load_series(joint_msgs, JOINT_NAMES)
    trel = t - t0
    if not args.full:
        keep = trel >= 0.0
        if keep.sum() < 2:
            print('warning: trajectory-dispatch window empty; plotting full bag')
        else:
            trel, pos, vel = trel[keep], pos[keep], vel[keep]
    if not args.full and not args.no_trim and len(trel) > 2:
        # trim the trailing static hold (bag often runs past the last motion)
        motion = (np.nanmax(np.abs(vel), axis=1) if np.isfinite(vel).any()
                  else np.nanmax(np.abs(np.gradient(pos, trel, axis=0)), axis=1))
        moving = np.where(motion > args.static_eps)[0]
        if moving.size:
            tend = trel[moving[-1]] + args.tail_pad
            m2 = trel <= tend
            trel, pos, vel = trel[m2], pos[m2], vel[m2]
    print(f'plotting {len(trel)} joint samples over t=[{trel.min():.1f}, {trel.max():.1f}] s')

    # ── the four figures ────────────────────────────────────────────────────
    res41 = fig_path_overlay(args, urdf, cart_msgs, joint_msgs, chain, args.outdir, t0)
    fig_joint_positions(trel, pos, args.outdir, args.show)
    fig_joint_velocities(trel, vel, pos, args.outdir, args.show, args.vel_source)
    lims = joint_limit(urdf, 'joint_6')
    if lims is None:
        print('warning: no joint_6 limit in URDF — skipping Figure 4.5')
    else:
        j6lo, j6hi, mlo, mhi = fig_joint6_limits(trel, pos, lims, args.outdir, args.show)

    # ── console summary (handy for the thesis text) ─────────────────────────
    print('\n──── summary ────')
    if res41:
        print(f'Fig 4.1 tracking error: RMS {res41[0]:.2f} mm, max {res41[1]:.2f} mm')
    pk = np.nanmax(np.abs(vel), axis=0) if np.isfinite(vel).any() \
        else np.max(np.abs(np.gradient(pos, trel, axis=0)), axis=0)
    print('Fig 4.4 peak |velocity| per joint (rad/s): ' +
          ', '.join(f'{n}={v:.3f}' for n, v in zip(JOINT_NAMES, pk)))
    if lims is not None:
        print(f'Fig 4.5 joint_6 range used: [{j6lo:+.3f}, {j6hi:+.3f}] rad; '
              f'limits [{lims[0]:+.3f}, {lims[1]:+.3f}]; '
              f'closest approach {min(mlo, mhi):.3f} rad '
              f'→ {"WITHIN limits" if min(mlo, mhi) >= 0 else "EXCEEDED limits"}')


if __name__ == '__main__':
    main()
