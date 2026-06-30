#!/usr/bin/env python3
"""
gen_accuracy_csv.py — commanded vs measured accuracy comparison CSV.

Reads a rosbag2 recording and writes a time-aligned CSV comparing the
commanded joint positions against the measured encoder positions.

On real hardware:
  commanded = /joint_states  (what pos_motor_sub receives and sends to motors)
  measured  = /encoder_states (what pos_motor_sub reads back from motors)

In simulation:
  commanded = /joint_commands
  measured  = /joint_states

Output CSV columns:
  t_rel_s
  joint_1_cmd_rad .. joint_7_cmd_rad
  joint_1_meas_rad .. joint_7_meas_rad
  joint_1_err_rad  .. joint_7_err_rad   (cmd - meas)

Also prints per-joint RMS error to stdout.

Usage:
    python3 results_tools/gen_accuracy_csv.py <bag_dir>
    python3 results_tools/gen_accuracy_csv.py <bag_dir> --outdir figures/motion_capture
    python3 results_tools/gen_accuracy_csv.py <bag_dir> --out accuracy.csv
"""
import argparse
import csv
import os
import sys

import numpy as np

import data_io

JOINT_NAMES = [f'joint_{i}' for i in range(1, 8)]


def main():
    ap = argparse.ArgumentParser(description='Export commanded vs measured accuracy CSV from a bag.')
    ap.add_argument('bag', help='rosbag2 directory (contains bag.db3 or .mcap)')
    ap.add_argument('--outdir', default=None,
                    help='output directory (default: same as bag parent)')
    ap.add_argument('--out', default='accuracy.csv',
                    help='output filename (default: accuracy.csv)')
    ap.add_argument('--fps', type=float, default=100.0,
                    help='resample rate Hz (default: 100)')
    args = ap.parse_args()

    bag = args.bag
    if not os.path.isdir(bag):
        sys.exit(f'ERROR: bag directory not found: {bag}')

    cfg = data_io.load_config()
    names = JOINT_NAMES

    print(f'Reading bag: {bag}')
    data = data_io.read_bag(bag)

    # ── pick commanded stream ─────────────────────────────────────────────────
    # Real HW: encoder_states present → joint_states_commanded = original /joint_states
    # Sim / no encoder: fall back to joint_commands, then joint_states
    if 'joint_states_commanded' in data and data['joint_states_commanded']:
        cmd_msgs = data['joint_states_commanded']
        cmd_label = '/joint_states (commanded, real HW)'
    elif data['joint_commands']:
        cmd_msgs = data['joint_commands']
        cmd_label = '/joint_commands'
    elif data['joint_states']:
        cmd_msgs = data['joint_states']
        cmd_label = '/joint_states (only stream — no encoder feedback)'
    else:
        sys.exit('ERROR: no commanded joint data found in bag.')

    # ── pick measured stream ──────────────────────────────────────────────────
    meas_msgs = data['joint_states']   # already promoted from encoder_states if present
    meas_label = '/encoder_states (measured)' if data.get('encoder_states') else '/joint_states'

    if not meas_msgs:
        sys.exit('ERROR: no measured joint data found in bag.')

    print(f'  commanded : {cmd_label}  ({len(cmd_msgs)} msgs)')
    print(f'  measured  : {meas_label}  ({len(meas_msgs)} msgs)')

    # ── build shared timeline ─────────────────────────────────────────────────
    t_cmd, pos_cmd = data_io.jointstate_series(cmd_msgs, names)
    t_meas, pos_meas = data_io.jointstate_series(meas_msgs, names)

    t0 = data_io.time_origin(data, cfg)
    t_lo = max(t_cmd[0], t_meas[0])
    t_hi = min(t_cmd[-1], t_meas[-1])

    if t_lo >= t_hi:
        sys.exit('ERROR: commanded and measured time ranges do not overlap.')

    t_grid = data_io.make_timeline(t_lo, t_hi, args.fps)
    cmd_r = data_io.resample(t_cmd, pos_cmd, t_grid)
    meas_r = data_io.resample(t_meas, pos_meas, t_grid)
    err_r = cmd_r - meas_r
    t_rel = t_grid - t0

    # ── write CSV ─────────────────────────────────────────────────────────────
    outdir = args.outdir or os.path.dirname(os.path.abspath(bag))
    os.makedirs(outdir, exist_ok=True)
    out_path = os.path.join(outdir, args.out)

    header = ['t_rel_s']
    header += [f'{n}_cmd_rad' for n in names]
    header += [f'{n}_meas_rad' for n in names]
    header += [f'{n}_err_rad' for n in names]

    with open(out_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        for i in range(len(t_rel)):
            row = [f'{t_rel[i]:.6f}']
            row += [f'{v:.8f}' for v in cmd_r[i]]
            row += [f'{v:.8f}' for v in meas_r[i]]
            row += [f'{v:.8f}' for v in err_r[i]]
            w.writerow(row)

    print(f'\nWrote {len(t_rel)} rows → {out_path}')

    # ── per-joint RMS ─────────────────────────────────────────────────────────
    print('\nPer-joint RMS error (commanded − measured):')
    rms = np.sqrt(np.mean(err_r ** 2, axis=0))
    for n, r in zip(names, rms):
        print(f'  {n}: {r*1000:.3f} mrad  ({np.degrees(r):.4f} deg)')
    print(f'\n  Overall RMS (all joints): {np.sqrt(np.mean(err_r**2))*1000:.3f} mrad')


if __name__ == '__main__':
    main()
