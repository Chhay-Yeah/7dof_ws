#!/usr/bin/env python3
"""
capture.py — ONE command: record a draw, then auto-generate every artifact.

Workflow:

    source ~/7dof_ws/install/setup.bash
    python3 results_tools/capture.py          # <- run this FIRST
    # ... it starts recording. Hit Send so the robot draws ...
    # ... when the robot finishes, come back here and press Ctrl-C ...

On Ctrl-C it cleanly finalises the rosbag, then renders all three artifacts into
a single run folder under figures/:

    figures/draw_capture/
        bag/                       the raw recording (rosbag2)
        figure_4_1.{png,pdf}       commanded vs executed path overlay
        figure_4_6.{png,pdf}       seven joint velocities
        frontview/                 transparent frames + alpha .mov + ffmpeg cmds

Record again and the next folder is named "draw_capture (2)", "(3)", ... so
nothing is ever overwritten.

    python3 results_tools/capture.py --name square      # custom run name
    python3 results_tools/capture.py --no-anim          # skip the slow animation
"""
import argparse
import os
import signal
import subprocess
import sys
import time

import data_io

_HERE = os.path.dirname(os.path.abspath(__file__))
_PY = sys.executable


# ── unique " (2)" / " (3)" run folder ────────────────────────────────────────

def unique_dir(parent, name):
    base = os.path.join(parent, name)
    if not os.path.exists(base):
        return base
    i = 2
    while os.path.exists(f'{base} ({i})'):
        i += 1
    return f'{base} ({i})'


# ── pick feedback vs commanded from what was actually recorded ───────────────

def choose_drive(bag):
    try:
        data = data_io.read_bag(bag)
    except Exception:
        return 'feedback'
    if data['joint_states']:
        return 'feedback'
    if data['motor_target'] or data['joint_commands']:
        return 'commanded'
    return 'feedback'


# ── run a generator, report success/failure without aborting the others ──────

def run_step(title, cmd):
    print(f'\n── {title} ' + '─' * max(0, 50 - len(title)))
    try:
        rc = subprocess.call(cmd)
    except Exception as e:                       # noqa: BLE001
        print(f'   {title}: ERROR ({e})')
        return False
    if rc != 0:
        print(f'   {title}: generator exited {rc} (skipped — see message above)')
        return False
    return True


def main():
    cfg = data_io.load_config()
    rc = cfg['record']

    ap = argparse.ArgumentParser(description='Record a draw, then auto-generate all artifacts.')
    ap.add_argument('--name', default='draw_capture', help='run folder name (suffixed if it exists)')
    ap.add_argument('--storage', default=rc.get('storage', 'sqlite3'), choices=['sqlite3', 'mcap'])
    ap.add_argument('--topics', nargs='*', default=rc['topics'])
    ap.add_argument('--drive', default='auto', choices=['auto', 'feedback', 'commanded'],
                    help='which stream drives the front-view animation')
    ap.add_argument('--video', default=cfg['frontview']['export'].get('video', 'prores'),
                    choices=['prores', 'webm', 'none'])
    ap.add_argument('--fps', type=float, default=cfg['frontview'].get('fps', 30.0),
                    help='animation frame rate')
    ap.add_argument('--anim-max-frames', type=int, default=0, help='cap animation frames (0 = all)')
    ap.add_argument('--no-anim', action='store_true', help='skip the front-view animation')
    args = ap.parse_args()

    run_dir = unique_dir(data_io.figures_dir(cfg), args.name)
    os.makedirs(run_dir, exist_ok=True)
    bag = os.path.join(run_dir, 'bag')

    rec_cmd = ['ros2', 'bag', 'record', '-s', args.storage, '-o', bag] + args.topics
    print('=' * 64)
    print(f'RUN FOLDER: {run_dir}')
    print('RECORDING (observe-only). Topics:')
    print('   ' + '  '.join(args.topics))
    print('\n  >> Hit Send now so the robot draws.')
    print('  >> When the robot FINISHES, press Ctrl-C here to stop + generate.')
    print('=' * 64)

    # Start the recorder in its OWN session so the terminal's Ctrl-C reaches
    # ONLY this script; we then forward SIGINT to the recorder so rosbag2
    # finalises the file cleanly before we read it back.
    try:
        proc = subprocess.Popen(rec_cmd, start_new_session=True)
    except FileNotFoundError:
        sys.exit("ERROR: 'ros2' not found — source install/setup.bash first.")

    try:
        proc.wait()
        # recorder exited on its own (error?) — fall through to generation
    except KeyboardInterrupt:
        print('\n\nstopping recording…')
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=30)
        except Exception:                        # noqa: BLE001
            proc.kill()
    print('recording finalised.')

    if not os.path.isdir(bag):
        sys.exit(f'ERROR: no bag written at {bag} — nothing to generate.')

    # ── generate every artifact into the run folder ─────────────────────────
    time.sleep(0.3)
    drive = choose_drive(bag) if args.drive == 'auto' else args.drive
    print(f'\nGenerating artifacts (front-view drive = {drive})…')

    ok = []
    ok.append(('path overlay (fig 4.1)', run_step(
        'path overlay',
        [_PY, os.path.join(_HERE, 'gen_path_overlay.py'), bag, '--outdir', run_dir])))
    ok.append(('joint velocities (fig 4.6)', run_step(
        'joint velocities',
        [_PY, os.path.join(_HERE, 'gen_velocity.py'),
         '--replay', bag, '--export', '--outdir', run_dir])))
    if not args.no_anim:
        anim = [_PY, os.path.join(_HERE, 'gen_frontview_anim.py'), bag,
                '--outdir', os.path.join(run_dir, 'frontview'),
                '--drive', drive, '--video', args.video, '--fps', str(args.fps)]
        if args.anim_max_frames:
            anim += ['--max-frames', str(args.anim_max_frames)]
        ok.append(('front-view animation', run_step('front-view animation', anim)))

    # ── summary ─────────────────────────────────────────────────────────────
    print('\n' + '=' * 64)
    print(f'DONE — everything saved under: {run_dir}')
    for name, good in ok:
        print(f'   [{"ok" if good else "--"}] {name}')
    print('=' * 64)


if __name__ == '__main__':
    main()
