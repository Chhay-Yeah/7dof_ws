#!/usr/bin/env python3
"""
gen_path_anim.py — replay the drawing as it is produced over time.

Same data + paper-frame FK as gen_path_overlay.py (Figure 4.1), but instead of a
static overlay it ANIMATES the pen path: the line appears progressively, paced by
the recorded timestamps, so it looks like the drawing is being made in real time.
A moving dot marks the current pen position; the commanded path is shown faintly
as a reference guide. Exports an mp4 (ffmpeg) or an animated gif.

    source ~/7dof_ws/install/setup.bash
    python3 results_tools/gen_path_anim.py recordings/draw_capture.csv
    python3 results_tools/gen_path_anim.py run.csv --speed 2 --video gif
    python3 results_tools/gen_path_anim.py run.csv --start 30 --end 75
"""
import argparse
import importlib.util
import os
import sys

import numpy as np
import matplotlib

import data_io
import fk_chain

_HERE = os.path.dirname(os.path.abspath(__file__))
_P_PATH = os.path.join(_HERE, '..', 'src', 'arm_bot', 'analysis',
                       'plot_commanded_vs_executed.py')


def _load_helpers():
    spec = importlib.util.spec_from_file_location('pcve', _P_PATH)
    P = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(P)
    return P


def _paper_frame(P, po, chain):
    return P.PaperFrame(chain, np.array(po['begin_draw_joints'], float),
                        po['pen_offset_mm'] / 1000.0,
                        np.array(po['pen_axis_local'], float),
                        po['paper_rotation_deg'], po['paper_mirror_x'])


def _fk_to_plane(pf, chain, Q, frame):
    xs, ys, zs = [], [], []
    for q in Q:
        T = chain.fk(np.asarray(q, float))
        if frame == 'paper':
            x, y, z = pf.exec_to_paper_mm(T[:3, 3], T[:3, :3])
        else:
            x, y, z = pf.pen_tip(T[:3, 3], T[:3, :3]) * 1000.0
        xs.append(x); ys.append(y); zs.append(z)
    return np.array(xs), np.array(ys), np.array(zs)


def main():
    cfg = data_io.load_config()
    po = cfg['path_overlay']
    fkc = cfg['fk']

    ap = argparse.ArgumentParser(description='Animate the drawing path over time.')
    ap.add_argument('bag', help='encoder .csv OR rosbag2 dir')
    ap.add_argument('--outdir', default=data_io.figures_dir(cfg))
    ap.add_argument('--frame', default=po.get('frame', 'paper'), choices=['paper', 'base'])
    ap.add_argument('--start', type=float, default=None, help='trim start (t_rel s)')
    ap.add_argument('--end', type=float, default=None, help='trim end (t_rel s)')
    ap.add_argument('--speed', type=float, default=1.0,
                    help='playback speed multiplier (1.0 = real time)')
    ap.add_argument('--fps', type=float, default=30.0, help='output video frame rate')
    ap.add_argument('--video', default='mp4', choices=['mp4', 'gif'])
    ap.add_argument('--no-guide', action='store_true',
                    help='hide the faint commanded reference path')
    ap.add_argument('--show', action='store_true')
    args = ap.parse_args()

    if not args.show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter
    P = _load_helpers()

    # ── load executed + commanded pen paths in the paper plane ───────────────
    if data_io.is_csv(args.bag):
        urdf = data_io.get_urdf(cfg)
        chain = fk_chain.build_chain(urdf, fkc['base_link'], fkc['tip_link'])
        pf = _paper_frame(P, po, chain)
        d = data_io.read_csv(args.bag, fkc['joint_names'])
        m = np.all(np.isfinite(d['enc']), axis=1) & np.all(np.isfinite(d['cmd']), axis=1)
        m &= data_io.time_mask(d['t'], args.start, args.end)
        t = d['t'][m]
        cmd_x, cmd_y, cmd_z = _fk_to_plane(pf, chain, d['cmd'][m], args.frame)
        ex, ey, ez = _fk_to_plane(pf, chain, d['enc'][m], args.frame)
    else:
        data = data_io.read_bag(args.bag)
        urdf = data_io.get_urdf(cfg, data['urdf'])
        if not data['joint_states']:
            sys.exit('ERROR: no executed joint stream in bag.')
        chain = fk_chain.build_chain(urdf, fkc['base_link'], fkc['tip_link'])
        pf = _paper_frame(P, po, chain)
        t0 = data_io.time_origin(data, cfg)
        te = np.array([ts - t0 for ts, _ in data['joint_states']])
        Q = [P.joint_q(msg, chain.joint_names) for _, msg in data['joint_states']]
        w = data_io.time_mask(te, args.start, args.end)
        t = te[w]
        ex, ey, ez = _fk_to_plane(pf, chain, [Q[i] for i in np.where(w)[0]], args.frame)

        # commanded guide from /cartesian_path if present
        cmd_x = cmd_y = cmd_z = None
        if data['cartesian_path'] and data['cartesian_path'][-1].poses:
            pa = data['cartesian_path'][-1]
            cmd_arr = np.array([[ps.position.x, ps.position.y, ps.position.z]
                                for ps in pa.poses]) * 1000.0
            cmd_x, cmd_y, cmd_z = cmd_arr[:, 0], cmd_arr[:, 1], cmd_arr[:, 2]

    t = t - t[0]

    # separate pen-up thresholds per stream (measured z may be offset from commanded)
    exec_up = (po['pen_up_mm'] if po.get('pen_up_mm') is not None
               else P.auto_pen_up_thresh(ez))
    fx, fy = P.break_on_pen_up(ex, ey, ez, exec_up)

    # remove registration offset so measured path aligns with commanded guide
    if cmd_x is not None:
        cmd_up = P.auto_pen_up_thresh(cmd_z)
        f_down = np.isfinite(fx) & np.isfinite(fy)
        c_down = cmd_z <= cmd_up
        if f_down.any() and c_down.any():
            dx = np.nanmean(fx[f_down]) - np.nanmean(cmd_x[c_down])
            dy = np.nanmean(fy[f_down]) - np.nanmean(cmd_y[c_down])
            fx = fx - dx
            fy = fy - dy

    # ── frame schedule: one frame per 1/fps of (scaled) recorded time ────────
    duration = max(1e-3, (t[-1] - t[0]) / max(args.speed, 1e-6))
    n_frames = max(2, int(round(duration * args.fps)))
    frame_t = np.linspace(t[0], t[-1], n_frames)
    reveal = np.searchsorted(t, frame_t, side='right')   # samples visible per frame

    # ── figure ───────────────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    pad = 5.0
    xs_all = fx[np.isfinite(fx)]; ys_all = fy[np.isfinite(fy)]
    ax.set_xlim(xs_all.min() - pad, xs_all.max() + pad)
    ax.set_ylim(ys_all.min() - pad, ys_all.max() + pad)
    ax.set_aspect('equal'); ax.grid(True, ls=':', alpha=0.5)
    ax.set_xlabel(f'{args.frame} X (mm)'); ax.set_ylabel(f'{args.frame} Y (mm)')
    ax.set_title('Drawing playback')
    if cmd_x is not None and not args.no_guide:
        _cup = P.auto_pen_up_thresh(cmd_z) if cmd_z is not None else exec_up
        gx, gy = P.break_on_pen_up(cmd_x, cmd_y, cmd_z, _cup)
        ax.plot(gx, gy, color='0.7', lw=1.0, ls='--', label='commanded (guide)', zorder=1)
    (drawn,) = ax.plot([], [], color='tab:red', lw=2.0, label='drawn', zorder=2)
    (pen,) = ax.plot([], [], 'o', color='tab:red', ms=8, zorder=3)
    clock = ax.text(0.02, 0.98, '', transform=ax.transAxes, va='top', ha='left',
                    fontsize=10, bbox=dict(boxstyle='round', fc='white', alpha=0.8))
    ax.legend(loc='lower right', fontsize=9)

    def update(i):
        k = int(reveal[i])
        drawn.set_data(fx[:k], fy[:k])
        if k > 0 and np.isfinite(fx[k - 1]):
            pen.set_data([fx[k - 1]], [fy[k - 1]])
        else:
            pen.set_data([], [])
        clock.set_text(f't = {frame_t[i]:5.2f} s')
        return drawn, pen, clock

    anim = FuncAnimation(fig, update, frames=n_frames, interval=1000.0 / args.fps,
                         blit=True)

    os.makedirs(args.outdir, exist_ok=True)
    stem = po['export_stem'] + '_anim'
    if args.show:
        plt.show()
        return
    if args.video == 'gif':
        out = os.path.join(args.outdir, stem + '.gif')
        anim.save(out, writer=PillowWriter(fps=args.fps))
    else:
        out = os.path.join(args.outdir, stem + '.mp4')
        anim.save(out, writer=FFMpegWriter(fps=args.fps, bitrate=2400))
    print(f'wrote {out}  ({n_frames} frames, {duration:.1f}s @ {args.fps:g}fps, '
          f'speed x{args.speed:g})')


if __name__ == '__main__':
    main()
