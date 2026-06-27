#!/usr/bin/env python3
"""
gen_frontview_anim.py — Deliverable 3: front-view robot animation overlay.

A 2D front-view kinematic rendering: the single shared FK (fk_chain.Chain) gives
every joint's 3D position, which is projected onto a configurable front-view
plane and drawn as connected segments with joint markers, optionally trailing
the pen-tip trace. Exports a TRANSPARENT PNG frame sequence (with alpha) and an
alpha video (ProRes 4444 .mov or VP9 .webm), plus the exact ffmpeg command to
composite it over your recorded clip.

Drive it from EITHER the recorded encoder feedback (proves the digital twin
matches the encoders) OR the commanded setpoints (proves the real arm tracks
the target) — config frontview.drive, or --drive.

    source ~/7dof_ws/install/setup.bash
    python3 results_tools/gen_frontview_anim.py draw_run
    python3 results_tools/gen_frontview_anim.py draw_run --drive commanded

Everything is resampled onto a uniform timeline at frontview.fps (default 30,
match your video) so frames line up with the other artifacts.
"""
import argparse
import os
import subprocess
import sys

import numpy as np

import data_io
import fk_chain

AXIS = {'x': 0, 'y': 1, 'z': 2}


# ── projection: 3D base-frame point -> 2D screen pixels ──────────────────────

class Projector:
    """Drop the world axis pointing into the screen; map the other two to
    screen-right / screen-up; apply flip, in-plane rotation, scale, origin."""

    def __init__(self, fv):
        pj = fv['projection']
        rn = fv['render']
        self.ix = AXIS[pj['screen_x']]
        self.iy = AXIS[pj['screen_y']]
        self.sx = -1.0 if pj.get('flip_x') else 1.0
        self.sy = -1.0 if pj.get('flip_y') else 1.0
        th = np.radians(pj.get('rotation_deg', 0.0))
        self.c, self.s = float(np.cos(th)), float(np.sin(th))
        self.scale = float(rn['scale_px_per_m'])
        self.ox, self.oy = rn['origin_px']

    def __call__(self, P3):
        """P3: (...,3) base-frame metres -> (...,2) pixels (image coords)."""
        P3 = np.atleast_2d(P3)
        u = self.sx * P3[:, self.ix]
        v = self.sy * P3[:, self.iy]
        ur = u * self.c - v * self.s
        vr = u * self.s + v * self.c
        px = self.ox + self.scale * ur
        py = self.oy - self.scale * vr      # image y grows downward
        return np.column_stack([px, py])


# ── joint stream selection ───────────────────────────────────────────────────

def joint_stream(data, cfg, drive):
    names = cfg['fk']['joint_names']
    if drive == 'feedback':
        if not data['joint_states']:
            sys.exit('ERROR: drive=feedback but no /joint_states in bag.')
        return data_io.jointstate_series(data['joint_states'], names)
    # commanded
    topic = cfg['frontview'].get('commanded_topic', '/motor_target')
    if topic == '/motor_target' and data['motor_target']:
        return data_io.motortarget_series(data['motor_target'], names)
    if topic == '/joint_commands' and data['joint_commands']:
        return data_io.jointstate_series(data['joint_commands'], names)
    # fall back to whichever commanded stream exists
    if data['motor_target']:
        print(f'note: {topic} empty — using /motor_target')
        return data_io.motortarget_series(data['motor_target'], names)
    if data['joint_commands']:
        print(f'note: {topic} empty — using /joint_commands')
        return data_io.jointstate_series(data['joint_commands'], names)
    sys.exit(f'ERROR: drive=commanded but no {topic} (nor any commanded stream) in bag.')


# ── ffmpeg command strings ───────────────────────────────────────────────────

def video_cmd(kind, fps, frames_glob, out):
    if kind == 'prores':
        return ['ffmpeg', '-y', '-framerate', str(fps), '-i', frames_glob,
                '-c:v', 'prores_ks', '-profile:v', '4444',
                '-pix_fmt', 'yuva444p10le', out]
    if kind == 'webm':
        return ['ffmpeg', '-y', '-framerate', str(fps), '-i', frames_glob,
                '-c:v', 'libvpx-vp9', '-pix_fmt', 'yuva420p', '-b:v', '0',
                '-crf', '24', out]
    return None


def composite_cmd(video_path, fps):
    return (f'ffmpeg -i YOUR_CLIP.mp4 -i {video_path} '
            f'-filter_complex "[0:v][1:v]overlay=0:0:format=auto" '
            f'-c:a copy -r {fps} composited.mp4')


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    cfg = data_io.load_config()
    fv = cfg['frontview']
    fkc = cfg['fk']
    po = cfg['path_overlay']

    ap = argparse.ArgumentParser(description='Front-view robot animation overlay.')
    ap.add_argument('bag', help='rosbag2 dir with the recorded run')
    ap.add_argument('--drive', default=fv.get('drive', 'feedback'),
                    choices=['feedback', 'commanded'])
    ap.add_argument('--fps', type=float, default=fv.get('fps', 30.0))
    ap.add_argument('--outdir', default=None, help='override output dir')
    ap.add_argument('--video', default=fv['export'].get('video', 'prores'),
                    choices=['prores', 'webm', 'none'])
    ap.add_argument('--max-frames', type=int, default=0,
                    help='cap frame count (0 = all)')
    ap.add_argument('--start', type=float, default=None, help='t_rel start (s)')
    ap.add_argument('--end', type=float, default=None, help='t_rel end (s)')
    args = ap.parse_args()

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    if data_io.is_csv(args.bag):
        d = data_io.read_csv(args.bag, fkc['joint_names'])
        urdf = data_io.get_urdf(cfg)
        raw = d['enc'] if args.drive == 'feedback' else d['cmd']
        m = np.all(np.isfinite(raw), axis=1)
        trel, pos = d['t'][m], raw[m]
    else:
        data = data_io.read_bag(args.bag)
        urdf = data_io.get_urdf(cfg, data['urdf'])
        t, pos = joint_stream(data, cfg, args.drive)
        trel = t - data_io.time_origin(data, cfg)

    if urdf is None:
        sys.exit('ERROR: no URDF available (no /robot_description, no cache, no xacro).')
    chain = fk_chain.build_chain(urdf, fkc['base_link'], fkc['tip_link'])
    if trel.size < 2:
        sys.exit('ERROR: fewer than 2 joint samples in the selected stream.')
    lo = args.start if args.start is not None else float(trel.min())
    hi = args.end if args.end is not None else float(trel.max())
    grid = data_io.make_timeline(lo, hi, args.fps)
    if args.max_frames and len(grid) > args.max_frames:
        grid = grid[:args.max_frames]
    pos_g = data_io.resample(trel, pos, grid)

    proj = Projector(fv)
    rn = fv['render']
    out_root = args.outdir or os.path.join(data_io.figures_dir(cfg),
                                           fv['export'].get('out_subdir', 'frontview'))
    frames_dir = os.path.join(out_root, 'frames')
    os.makedirs(frames_dir, exist_ok=True)

    # pen-tip (for the accumulated trace), reusing the overlay pen params
    pen_off = po['pen_offset_mm'] / 1000.0
    pen_axis = np.array(po['pen_axis_local'], float)
    pen_axis = pen_axis / np.linalg.norm(pen_axis)

    w, h = rn['width'], rn['height']
    dpi = 100.0
    trace_px = []
    print(f'rendering {len(grid)} frames @ {args.fps} fps  ({args.drive}) -> {frames_dir}')

    for i, q in enumerate(pos_g):
        _, P3 = chain.fk_links(q)
        px = proj(P3)
        # pen tip = ee pose + offset along pen axis (base frame), then project
        T = chain.fk(q)
        tip3 = T[:3, 3] + pen_off * (T[:3, :3] @ pen_axis)
        tip_px = proj(tip3)[0]
        if fv.get('trace_pen_tip', True):
            trace_px.append(tip_px)

        fig = plt.figure(figsize=(w / dpi, h / dpi), dpi=dpi)
        fig.patch.set_alpha(0.0)
        ax = fig.add_axes([0, 0, 1, 1])
        ax.set_xlim(0, w); ax.set_ylim(h, 0)   # invert y -> image coords
        ax.axis('off'); ax.patch.set_alpha(0.0)
        if len(trace_px) >= 2:
            tr = np.array(trace_px)
            ax.plot(tr[:, 0], tr[:, 1], '-', color=rn['pen_trace_color'],
                    lw=rn['pen_trace_thickness'], solid_capstyle='round', zorder=2)
        ax.plot(px[:, 0], px[:, 1], '-', color=rn['arm_color'],
                lw=rn['line_thickness'], solid_capstyle='round',
                solid_joinstyle='round', zorder=3)
        ax.plot(px[:, 0], px[:, 1], 'o', color=rn['joint_color'],
                ms=rn['joint_marker_size'], zorder=4)
        fig.savefig(os.path.join(frames_dir, f'frame_{i:05d}.png'),
                    transparent=True, dpi=dpi)
        plt.close(fig)
        if i % 50 == 0:
            print(f'  {i}/{len(grid)}')

    print(f'wrote {len(grid)} transparent PNG frames')

    # ── alpha video ──────────────────────────────────────────────────────────
    frames_glob = os.path.join(frames_dir, 'frame_%05d.png')
    ext = 'mov' if args.video == 'prores' else 'webm'
    video_path = os.path.join(out_root, f'frontview_{args.drive}.{ext}')
    have_ffmpeg = _which('ffmpeg')
    if args.video != 'none':
        cmd = video_cmd(args.video, args.fps, frames_glob, video_path)
        if have_ffmpeg:
            print('encoding alpha video:\n  ' + ' '.join(cmd))
            rc = subprocess.call(cmd)
            if rc == 0:
                print(f'wrote {video_path}')
            else:
                print('ffmpeg failed — frames are still on disk.')
        else:
            print('\nffmpeg not installed — encode the alpha video yourself:')
            print('  ' + ' '.join(cmd))

    # ── composite-over-your-clip command ─────────────────────────────────────
    print('\ncomposite the overlay over your recorded video with:')
    print('  ' + composite_cmd(video_path, args.fps))
    _write_ffmpeg_notes(out_root, args, frames_glob, video_path,
                        video_cmd(args.video, args.fps, frames_glob, video_path))


def _write_ffmpeg_notes(out_root, args, frames_glob, video_path, vcmd):
    p = os.path.join(out_root, 'ffmpeg_commands.txt')
    with open(p, 'w') as f:
        f.write('# Front-view overlay — ffmpeg commands\n\n')
        if vcmd:
            f.write('# 1) encode the transparent frames into an alpha video:\n')
            f.write(' '.join(vcmd) + '\n\n')
        f.write('# 2) composite the overlay on top of your recorded clip:\n')
        f.write(composite_cmd(video_path, args.fps) + '\n')
    print(f'wrote {p}')


def _which(prog):
    from shutil import which
    return which(prog) is not None


if __name__ == '__main__':
    main()
