#!/usr/bin/env python3
"""
gen_error_time.py — tracking error vs time.

Same data as figure_4.1 (commanded path vs measured encoder path, centroid-
aligned, pen-down only), but plotted as error (mm) on the Y axis and time (s)
from drawing start on the X axis.

    python3 results_tools/gen_error_time.py figures/draw_capture/bag
    python3 results_tools/gen_error_time.py figures/draw_capture/bag --show
"""
import argparse
import importlib.util
import os
import sys

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

import data_io
import fk_chain

_P_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       '..', 'src', 'arm_bot', 'analysis',
                       'plot_commanded_vs_executed.py')


def _load_P():
    spec = importlib.util.spec_from_file_location('pcve', _P_PATH)
    P = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(P)
    return P


def main():
    cfg = data_io.load_config()
    po  = cfg['path_overlay']
    fkc = cfg['fk']

    ap = argparse.ArgumentParser()
    ap.add_argument('bag', help='rosbag2 directory')
    ap.add_argument('--outdir', default=data_io.figures_dir(cfg))
    ap.add_argument('--show', action='store_true')
    args = ap.parse_args()

    if not args.show:
        matplotlib.use('Agg')

    P = _load_P()

    data = data_io.read_bag(args.bag)
    urdf = data_io.get_urdf(cfg, data['urdf'])
    if not urdf:
        sys.exit('no URDF')
    if not data['joint_states']:
        sys.exit('no joint states in bag')

    chain = fk_chain.build_chain(urdf, fkc['base_link'], fkc['tip_link'])

    # decide mode: drawing (cartesian_path with poses) vs motion-only
    has_path = (data['cartesian_path'] and
                len(data['cartesian_path'][-1].poses) > 0)

    if has_path:
        # ── DRAWING MODE: nearest-point to commanded polyline (paper frame) ──
        pa  = data['cartesian_path'][-1]
        cmd = np.array([[p.position.x, p.position.y, p.position.z]
                        for p in pa.poses]) * 1000.0
        cmd_x, cmd_y, cmd_z = cmd[:, 0], cmd[:, 1], cmd[:, 2]

        begin    = np.array(po['begin_draw_joints'], float)
        pen_axis = np.array(po['pen_axis_local'], float)
        pf = P.PaperFrame(chain, begin, po['pen_offset_mm'] / 1000.0,
                          pen_axis, po['paper_rotation_deg'], po['paper_mirror_x'])

        t0         = data_io.time_origin(data, cfg)
        draw_start = t0 + po['move_to_begin'] + po['dwell'] + po['settle'] + po['approach']

        ts_list, ex_list, ey_list, ez_list = [], [], [], []
        for ts, msg in data['joint_states']:
            if ts < draw_start:
                continue
            try:
                q = P.joint_q(msg, chain.joint_names)
            except ValueError:
                continue
            T = chain.fk(q)
            x, y, z = pf.exec_to_paper_mm(T[:3, 3], T[:3, :3])
            ts_list.append(ts); ex_list.append(x); ey_list.append(y); ez_list.append(z)

        ex = np.array(ex_list); ey = np.array(ey_list)
        ez = np.array(ez_list); ts = np.array(ts_list)
        t_rel = ts - ts[0]

        cmd_up  = P.auto_pen_up_thresh(cmd_z)
        exec_up = P.auto_pen_up_thresh(ez)
        f_down  = ez <= exec_up
        c_down  = cmd_z <= cmd_up
        if f_down.any() and c_down.any():
            dx = np.nanmean(ex[f_down]) - np.nanmean(cmd_x[c_down])
            dy = np.nanmean(ey[f_down]) - np.nanmean(cmd_y[c_down])
            ex -= dx; ey -= dy

        pen_down = ez <= exec_up
        cx, cy = P.break_on_pen_up(cmd_x, cmd_y, cmd_z, cmd_up)
        cm = np.isfinite(cx) & np.isfinite(cy)
        poly_x, poly_y = cx[cm], cy[cm]

        errors = np.full(len(ex), np.nan)
        for i in range(len(ex)):
            if not pen_down[i]:
                continue
            errors[i] = P.point_to_polyline_mm(ex[i], ey[i], poly_x, poly_y)

        ylabel = 'Tracking error (mm)'
        mode_label = 'commanded path vs measured (nearest-point)'

    else:
        # ── MOTION MODE: nearest-point on 3-D commanded EE path ──
        # Same idea as drawing mode but in 3-D base-frame mm instead of paper mm.
        # For each measured EE sample, find the nearest point on the commanded EE
        # polyline — immune to timing offsets between the two streams.
        cmd_msgs  = data.get('joint_states_commanded') or data['joint_states']
        meas_msgs = data['joint_states']

        t_cmd,  q_cmd  = data_io.jointstate_series(cmd_msgs,  fkc['joint_names'])
        t_meas, q_meas = data_io.jointstate_series(meas_msgs, fkc['joint_names'])

        # FK the commanded joints → 3-D EE path (mm)
        cmd_pts = np.array([chain.fk(q)[:3, 3] * 1000.0 for q in q_cmd])  # (M, 3)

        # trim measured to the overlap window so pre-start samples are excluded
        overlap = (t_meas >= t_cmd[0]) & (t_meas <= t_cmd[-1])
        t_meas  = t_meas[overlap]
        q_meas  = q_meas[overlap]

        # FK the measured joints → 3-D EE path (mm), with timestamps
        meas_pts = np.array([chain.fk(q)[:3, 3] * 1000.0 for q in q_meas])  # (N, 3)
        t_rel = t_meas - t_meas[0]

        # per-sample nearest-point distance to commanded 3-D polyline
        def _nearest_3d(pt, poly):
            """Min distance from pt (3,) to 3-D polyline poly (M, 3)."""
            a = poly[:-1]; b = poly[1:]
            ab = b - a
            ap = pt - a
            t_ = np.einsum('ij,ij->i', ap, ab) / np.maximum(
                    np.einsum('ij,ij->i', ab, ab), 1e-12)
            t_ = np.clip(t_, 0.0, 1.0)
            closest = a + t_[:, None] * ab
            return float(np.min(np.linalg.norm(pt - closest, axis=1)))

        errors = np.array([_nearest_3d(pt, cmd_pts) for pt in meas_pts])

        # trim leading sync artifact: skip samples before the arm first settles
        # onto the commanded path (error first drops below 5 mm)
        settled = np.where(errors < 5.0)[0]
        if settled.size:
            start_idx = settled[0]
            errors = errors[start_idx:]
            t_rel  = t_rel[start_idx:] - t_rel[start_idx]

        ylabel = 'EE position error (mm)'
        mode_label = 'measured EE nearest-point on commanded 3-D path'

    pd_err = errors[np.isfinite(errors)]
    rms = float(np.sqrt(np.mean(pd_err**2)))
    mx  = float(np.max(pd_err))
    print(f'mode: {"drawing" if has_path else "motion"}')
    print(f'samples: {len(pd_err)}   RMS: {rms:.3f} mm   max: {mx:.3f} mm')

    # ── plot ──────────────────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(10, 4))

    t_valid = t_rel[np.isfinite(errors)]
    e_valid = errors[np.isfinite(errors)]

    ax.plot(t_valid, e_valid, color='#1f77b4', lw=1.0)
    ax.axhline(rms, ls='--', color='#d62728', lw=1.2, label=f'RMS {rms:.2f} mm')
    ax.axhline(mx,  ls=':',  color='#ff7f0e', lw=1.2, label=f'max {mx:.2f} mm')

    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel(ylabel, fontsize=11)
    ax.set_xlim(t_valid[0], t_valid[-1])   # start at first data point, no leading gap
    ax.set_ylim(0)
    ax.grid(True, ls=':', alpha=0.4)
    ax.set_title(f'Commanded vs measured tracking error over time — '
                 f'RMS {rms:.2f} mm, max {mx:.2f} mm', fontsize=11)
    ax.legend(loc='lower right', fontsize=10)
    fig.tight_layout()

    os.makedirs(args.outdir, exist_ok=True)
    out = os.path.join(args.outdir, 'error_vs_time.png')
    fig.savefig(out, dpi=300)
    fig.savefig(out.replace('.png', '.pdf'))
    print(f'wrote {out}')
    if args.show:
        plt.show()


if __name__ == '__main__':
    main()
