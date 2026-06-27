#!/usr/bin/env python3
"""
gen_from_csv.py — ONE command: turn a record_encoder.py CSV into every artifact.

    python3 results_tools/gen_from_csv.py recordings/draw_capture.csv

Renders, into a single run folder under figures/ (suffixed " (2)", " (3)" so
nothing is overwritten):

    figure_4_1.{png,pdf}   commanded (FK of commanded joints) vs executed (FK of
                           the measured encoder), pen path + RMS/max error
    figure_4_6.{png,pdf}   seven joint velocities — the MEASURED encoder velocity
    frontview/             front-view animation from the measured encoder

Drive the animation from the measured encoder (default) or the commanded joints
with --drive. Skip the slow animation with --no-anim.
"""
import argparse
import os
import sys

import data_io
from capture import unique_dir, run_step          # reuse the same helpers

_HERE = os.path.dirname(os.path.abspath(__file__))
_PY = sys.executable


def main():
    cfg = data_io.load_config()
    ap = argparse.ArgumentParser(description='Generate all artifacts from an encoder CSV.')
    ap.add_argument('csv', help='encoder CSV from record_encoder.py')
    ap.add_argument('--name', default=None, help='run folder name (default: CSV basename)')
    ap.add_argument('--drive', default='feedback', choices=['feedback', 'commanded'],
                    help='front-view animation source (feedback=encoder, commanded=cmd)')
    ap.add_argument('--video', default=cfg['frontview']['export'].get('video', 'prores'),
                    choices=['prores', 'webm', 'none'])
    ap.add_argument('--fps', type=float, default=cfg['frontview'].get('fps', 30.0))
    ap.add_argument('--anim-max-frames', type=int, default=0)
    ap.add_argument('--diff', action='store_true',
                    help='velocity from differentiated position instead of measured encoder velocity')
    ap.add_argument('--no-anim', action='store_true')
    args = ap.parse_args()

    if not os.path.exists(args.csv):
        sys.exit(f'ERROR: no such CSV: {args.csv}')
    csv = os.path.abspath(args.csv)
    name = args.name or os.path.splitext(os.path.basename(csv))[0]
    run_dir = unique_dir(data_io.figures_dir(cfg), name)
    os.makedirs(run_dir, exist_ok=True)
    print(f'RUN FOLDER: {run_dir}\nsource CSV: {csv}\n')

    ok = []
    ok.append(('path overlay (fig 4.1)', run_step(
        'path overlay',
        [_PY, os.path.join(_HERE, 'gen_path_overlay.py'), csv, '--outdir', run_dir])))

    vel = [_PY, os.path.join(_HERE, 'gen_velocity.py'), '--replay', csv, '--export', '--outdir', run_dir]
    if args.diff:
        vel.append('--diff')
    ok.append(('joint velocities (fig 4.6)', run_step('joint velocities', vel)))

    if not args.no_anim:
        anim = [_PY, os.path.join(_HERE, 'gen_frontview_anim.py'), csv,
                '--outdir', os.path.join(run_dir, 'frontview'),
                '--drive', args.drive, '--video', args.video, '--fps', str(args.fps)]
        if args.anim_max_frames:
            anim += ['--max-frames', str(args.anim_max_frames)]
        ok.append(('front-view animation', run_step('front-view animation', anim)))

    print('\n' + '=' * 64)
    print(f'DONE — everything saved under: {run_dir}')
    for nm, good in ok:
        print(f'   [{"ok" if good else "--"}] {nm}')
    print('=' * 64)


if __name__ == '__main__':
    main()
