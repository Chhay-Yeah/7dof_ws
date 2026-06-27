#!/usr/bin/env python3
"""
gen_path_overlay.py — Deliverable 1: commanded vs executed drawing path.

  commanded = the /cartesian_path waypoints the planner dispatched (paper frame)
  executed  = FK of the recorded ENCODER /joint_states, mapped into that same
              paper plane via the pen-tip transform

Overlays the two and reports RMS / max tracking error in mm. This is thesis
Figure 4.1. FK comes from the single shared chain (fk_chain.Chain); the paper
transform + tracking-error + plotting reuse the validated helpers in
arm_bot/analysis/plot_commanded_vs_executed.py.

    source ~/7dof_ws/install/setup.bash
    python3 results_tools/gen_path_overlay.py draw_run            # replay a bag
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
    begin = np.array(po['begin_draw_joints'], float)
    pen_axis = np.array(po['pen_axis_local'], float)
    return P.PaperFrame(chain, begin, po['pen_offset_mm'] / 1000.0, pen_axis,
                        po['paper_rotation_deg'], po['paper_mirror_x'])


def _fk_to_plane(pf, chain, Q, frame):
    """FK each joint vector in Q to pen-tip (x,y,z) in mm — paper or base frame."""
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

    ap = argparse.ArgumentParser(description='Figure 4.1 — commanded vs executed path.')
    ap.add_argument('bag', help='rosbag2 dir (with /cartesian_path) OR an encoder .csv '
                                '(commanded = FK of commanded joints)')
    ap.add_argument('--outdir', default=data_io.figures_dir(cfg))
    ap.add_argument('--frame', default=po.get('frame', 'paper'), choices=['paper', 'base'])
    ap.add_argument('--no-time-window', action='store_true',
                    help='plot the whole executed bag, not just the draw window')
    ap.add_argument('--show', action='store_true')
    args = ap.parse_args()

    if not args.show:
        matplotlib.use('Agg')
    P = _load_helpers()

    csv_mode = data_io.is_csv(args.bag)
    chain = None

    if csv_mode:
        urdf = data_io.get_urdf(cfg)
        chain = fk_chain.build_chain(urdf, fkc['base_link'], fkc['tip_link'])
        pf = _paper_frame(P, po, chain)
        d = data_io.read_csv(args.bag, fkc['joint_names'])
        m = np.all(np.isfinite(d['enc']), axis=1) & np.all(np.isfinite(d['cmd']), axis=1)
        # commanded = FK of commanded joints; executed = FK of measured encoder
        cmd_x, cmd_y, cmd_z = _fk_to_plane(pf, chain, d['cmd'][m], args.frame)
        ex, ey, ez = _fk_to_plane(pf, chain, d['enc'][m], args.frame)
    else:
        data = data_io.read_bag(args.bag)
        urdf = data_io.get_urdf(cfg, data['urdf'])
        if urdf is None:
            sys.exit('ERROR: no URDF (no /robot_description, no cache, no xacro).')
        if not data['joint_states']:
            sys.exit('ERROR: no /joint_states in bag.')
        if not data['cartesian_path']:
            sys.exit('ERROR: no /cartesian_path in bag — nothing to compare against.')
        chain = fk_chain.build_chain(urdf, fkc['base_link'], fkc['tip_link'])
        pf = _paper_frame(P, po, chain)

        pa = data['cartesian_path'][-1]
        cmd = np.array([[ps.position.x, ps.position.y, ps.position.z] for ps in pa.poses])
        t0 = data_io.time_origin(data, cfg)
        draw_start = (t0 + po['move_to_begin'] + po['dwell'] + po['settle'] + po['approach']
                      if not args.no_time_window else -np.inf)

        Q = [P.joint_q(msg, chain.joint_names)
             for ts, msg in data['joint_states'] if ts >= draw_start]
        ex, ey, ez = _fk_to_plane(pf, chain, Q, args.frame)
        if args.frame == 'paper':
            cmd_xyz = cmd * 1000.0
        else:
            cmd_xyz = np.array([pf.cmd_paper_to_base_mm(p) for p in cmd])
        cmd_x, cmd_y, cmd_z = cmd_xyz[:, 0], cmd_xyz[:, 1], cmd_xyz[:, 2]

    up = (po['pen_up_mm'] if po.get('pen_up_mm') is not None
          else P.auto_pen_up_thresh(cmd_z))
    cx, cy = P.break_on_pen_up(cmd_x, cmd_y, cmd_z, up)
    fx, fy = P.break_on_pen_up(ex, ey, ez, up)
    rms, mx, n = P.tracking_error(fx, fy, cx, cy)

    os.makedirs(args.outdir, exist_ok=True)
    out = os.path.join(args.outdir, po['export_stem'] + '.png')
    P.make_figure(cx, cy, fx, fy, out, args.frame, rms, mx, show=args.show)
    print(f'\nexecuted samples scored: {n}')
    if np.isfinite(rms):
        print(f'tracking error: RMS {rms:.3f} mm, max {mx:.3f} mm')


if __name__ == '__main__':
    main()
