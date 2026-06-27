#!/usr/bin/env python3
"""
record_draw.py — decoupled recorder for a drawing run.

Subscribes/observes only: this is a thin wrapper around `ros2 bag record`, so it
CANNOT perturb the draw (no commands published, no nodes modified). Start it
BEFORE you click Send / launch the MATLAB stroke so it catches the one-shot
/cartesian_path, then stop it (Ctrl-C) when the arm finishes.

    source ~/7dof_ws/install/setup.bash
    python3 results_tools/record_draw.py --name square_run

Writes a rosbag2 directory under record.output_dir (default draw_runs/). Replay
any of the three generators against it later. Topic list comes from config.yaml.
"""
import argparse
import os
import subprocess
import sys

from data_io import load_config


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    cfg = load_config()
    rc = cfg['record']

    ap = argparse.ArgumentParser(description='Record a drawing run (decoupled).')
    ap.add_argument('--name', default='draw_run',
                    help='output bag name (a numeric suffix is added if it exists)')
    ap.add_argument('--outdir', default=rc.get('output_dir', 'draw_runs'))
    ap.add_argument('--storage', default=rc.get('storage', 'sqlite3'),
                    choices=['sqlite3', 'mcap'])
    ap.add_argument('--topics', nargs='*', default=rc['topics'],
                    help='override the recorded topic list')
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    out = os.path.join(args.outdir, args.name)
    n = out
    i = 1
    while os.path.exists(n):
        n = f'{out}_{i}'
        i += 1
    out = n

    cmd = ['ros2', 'bag', 'record', '-s', args.storage, '-o', out] + args.topics
    print('recording (decoupled — observe only):')
    print('  ' + ' '.join(cmd))
    print(f'\noutput: {out}')
    print('\nstart this BEFORE the draw so /cartesian_path (one-shot) is caught.')
    print('stop with Ctrl-C when the arm finishes.\n')
    print('then generate the three artifacts with:')
    rel = os.path.relpath(out, here)
    print(f'  python3 {here}/gen_path_overlay.py    {out}')
    print(f'  python3 {here}/gen_velocity.py --replay {out}')
    print(f'  python3 {here}/gen_frontview_anim.py   {out}\n')

    try:
        return subprocess.call(cmd)
    except FileNotFoundError:
        sys.exit("ERROR: 'ros2' not found — source install/setup.bash first.")
    except KeyboardInterrupt:
        return 0


if __name__ == '__main__':
    sys.exit(main())
