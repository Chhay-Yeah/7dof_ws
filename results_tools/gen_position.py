#!/usr/bin/env python3
"""
gen_position.py — 7 individual joint position plots: commanded vs measured.

Static export (PNG + PDF):
    python3 results_tools/gen_position.py figures/pick\ and\ placce/bag --export

Animation (MP4 of lines being drawn in real time):
    python3 results_tools/gen_position.py figures/pick\ and\ placce/bag --animate

Both:
    python3 results_tools/gen_position.py figures/pick\ and\ placce/bag --export --animate
"""
import argparse
import os
import subprocess
import sys
import tempfile

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.figure import Figure

import data_io

NROWS, NCOLS = 4, 2
JOINT_COLORS = plt.cm.tab10.colors


# ── data loading ──────────────────────────────────────────────────────────────

def load(source, cfg):
    names = cfg['fk']['joint_names']
    fps   = cfg['timeline']['resample_fps']

    data  = data_io.read_bag(source)
    if not data['joint_states']:
        sys.exit('ERROR: no measured joint stream in bag.')
    t0 = data_io.time_origin(data, cfg)

    tm, pm = data_io.jointstate_series(data['joint_states'], names)
    grid   = data_io.make_timeline((tm - t0).min(), (tm - t0).max(), fps)
    meas   = data_io.resample(tm - t0, pm, grid)

    cmd_msgs = data.get('joint_states_commanded')
    if cmd_msgs:
        tc, pc = data_io.jointstate_series(cmd_msgs, names)
        cmd = data_io.resample(tc - t0, pc, grid)
    else:
        cmd = None

    return grid, cmd, meas, names


# ── figure helpers ─────────────────────────────────────────────────────────────

def make_fig():
    fig, axes = plt.subplots(NROWS, NCOLS, figsize=(11, 10), sharex=True)
    return fig, axes.ravel()


def draw_static(fig, axes, t, cmd, meas, names):
    for k, (ax, name) in enumerate(zip(axes, names)):
        if cmd is not None:
            ax.plot(t, cmd[:, k], lw=1.6, ls='--', color='0.45', label='commanded')
        ax.plot(t, meas[:, k], lw=1.4, color=JOINT_COLORS[k], label='measured')
        ax.set_title(name, fontsize=10)
        ax.grid(True, ls=':', alpha=0.5)
        if k % NCOLS == 0:
            ax.set_ylabel('position (rad)', fontsize=8)
        ax.tick_params(labelsize=7)

    for k in range(len(names), len(axes)):
        axes[k].axis('off')

    # x-labels on bottom row
    for k in (len(names) - 1, len(names) - NCOLS):
        if 0 <= k < len(names):
            axes[k].set_xlabel('time (s)', fontsize=8)

    # legend in spare cell or on first axes
    legend_ax = axes[len(names)] if len(names) < len(axes) else axes[0]
    handles, labels = axes[0].get_legend_handles_labels()
    legend_ax.legend(handles, labels, loc='center', fontsize=12, frameon=True)

    fig.suptitle('Joint position — commanded vs measured', fontsize=13)
    fig.tight_layout()


# ── static export ─────────────────────────────────────────────────────────────

def export_static(t, cmd, meas, names, outdir, stem):
    fig, axes = make_fig()
    draw_static(fig, axes, t, cmd, meas, names)
    os.makedirs(outdir, exist_ok=True)
    png = os.path.join(outdir, stem + '.png')
    pdf = os.path.join(outdir, stem + '.pdf')
    fig.savefig(png, dpi=150)
    fig.savefig(pdf)
    plt.close(fig)
    print(f'wrote {png} + {pdf}')


# ── animation ─────────────────────────────────────────────────────────────────

def export_animation(t, cmd, meas, names, outdir, stem, fps=30.0, speed=1.0):
    """Render frames where each joint's lines grow left-to-right in real time,
    then stitch into an MP4 with ffmpeg."""
    os.makedirs(outdir, exist_ok=True)

    duration   = t[-1] - t[0]          # seconds of data
    n_frames   = max(2, int(duration * fps / speed))
    frame_times = np.linspace(t[0], t[-1], n_frames)

    # Pre-compute full y-limits per joint so axes don't jump while animating
    ylims = []
    for k in range(len(names)):
        vals = [meas[:, k]]
        if cmd is not None:
            vals.append(cmd[:, k])
        all_v = np.concatenate(vals)
        pad = max(0.05, 0.1 * (all_v.max() - all_v.min()))
        ylims.append((all_v.min() - pad, all_v.max() + pad))

    with tempfile.TemporaryDirectory() as tmpdir:
        print(f'rendering {n_frames} frames…')
        for fi, ft in enumerate(frame_times):
            mask = t <= ft
            t_vis = t[mask]

            fig, axes = make_fig()
            for k, (ax, name) in enumerate(zip(axes, names)):
                if cmd is not None:
                    ax.plot(t_vis, cmd[mask, k], lw=1.6, ls='--', color='0.45',
                            label='commanded')
                ax.plot(t_vis, meas[mask, k], lw=1.4, color=JOINT_COLORS[k],
                        label='measured')
                ax.set_title(name, fontsize=10)
                ax.grid(True, ls=':', alpha=0.5)
                ax.set_xlim(t[0], t[-1])
                ax.set_ylim(*ylims[k])
                if k % NCOLS == 0:
                    ax.set_ylabel('position (rad)', fontsize=8)
                ax.tick_params(labelsize=7)

            for k in range(len(names), len(axes)):
                axes[k].axis('off')
            for k in (len(names) - 1, len(names) - NCOLS):
                if 0 <= k < len(names):
                    axes[k].set_xlabel('time (s)', fontsize=8)

            legend_ax = axes[len(names)] if len(names) < len(axes) else axes[0]
            handles, labels = axes[0].get_legend_handles_labels()
            legend_ax.legend(handles, labels, loc='center', fontsize=12, frameon=True)
            fig.suptitle('Joint position — commanded vs measured', fontsize=13)
            fig.tight_layout()

            frame_path = os.path.join(tmpdir, f'frame_{fi:05d}.png')
            fig.savefig(frame_path, dpi=100)
            plt.close(fig)

            if fi % 50 == 0:
                print(f'  {fi}/{n_frames}')

        mp4 = os.path.join(outdir, stem + '.mp4')
        cmd_ff = [
            'ffmpeg', '-y', '-r', str(fps),
            '-i', os.path.join(tmpdir, 'frame_%05d.png'),
            '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
            '-crf', '18', mp4
        ]
        print('encoding MP4…')
        subprocess.run(cmd_ff, check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f'wrote {mp4}')


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    cfg = data_io.load_config()
    ap = argparse.ArgumentParser(description='Per-joint position: commanded vs measured.')
    ap.add_argument('bag', help='rosbag2 directory')
    ap.add_argument('--outdir', default=None)
    ap.add_argument('--export', action='store_true', help='write static PNG + PDF')
    ap.add_argument('--animate', action='store_true', help='write real-time MP4 animation')
    ap.add_argument('--fps', type=float, default=30.0, help='animation frame rate')
    ap.add_argument('--speed', type=float, default=1.0,
                    help='playback speed multiplier (1.0 = real time, 2.0 = 2× faster)')
    ap.add_argument('--stem', default='figure_position', help='output filename stem')
    ap.add_argument('--trim-idle', action='store_true',
                    help='cut trailing idle tail where commanded joints are stationary')
    args = ap.parse_args()

    if not args.export and not args.animate:
        ap.error('specify --export, --animate, or both')

    outdir = args.outdir or data_io.figures_dir(cfg)
    t, cmd, meas, names = load(args.bag, cfg)

    if args.trim_idle and cmd is not None:
        # find last sample where any commanded joint moved by >0.005 rad (~0.3°)
        diffs = np.abs(np.diff(cmd, axis=0))
        moving_rows = np.where(np.any(diffs > 0.005, axis=1))[0]
        if moving_rows.size:
            cut = moving_rows[-1] + 2     # +1 for diff offset, +1 to include it
            cut = min(cut, len(t))
            t    = t[:cut]
            cmd  = cmd[:cut]
            meas = meas[:cut]
            print(f'trim-idle: cut at t={t[-1]:.3f}s ({cut} samples kept)')

    if args.export:
        export_static(t, cmd, meas, names, outdir, args.stem)

    if args.animate:
        export_animation(t, cmd, meas, names, outdir, args.stem,
                         fps=args.fps, speed=args.speed)


if __name__ == '__main__':
    main()
