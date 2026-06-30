#!/usr/bin/env python3
"""
gen_error_vs_time.py — tracking error over time during a drawing run.

For each sample in the CSV recording, computes the 2D Euclidean distance
(mm) between:
  • commanded pen-tip  = FK of the commanded joint positions  (paper frame)
  • measured  pen-tip  = FK of the encoder joint positions    (paper frame)

Plots that error signal from the start of the first pen-down stroke to the
end of the last pen-down stroke.  Pen-up (travel between strokes) is drawn
in grey; pen-down in red so you can see where the accuracy matters.

Usage:
    cd ~/7dof_ws
    source install/setup.bash
    python3 results_tools/gen_error_vs_time.py recordings/draw_capture.csv
    python3 results_tools/gen_error_vs_time.py recordings/draw_capture.csv --show
    python3 results_tools/gen_error_vs_time.py recordings/draw_capture.csv \\
        --start 30 --end 180   # manual trim (seconds relative to CSV t_rel_s=0)
"""
import argparse
import importlib.util
import os
import sys

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

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


def _fk_tips_paper(pf, chain, Q):
    """FK each row of Q → (x, y, z) pen-tip in paper frame (mm)."""
    xs, ys, zs = [], [], []
    for q in Q:
        T = chain.fk(np.asarray(q, float))
        x, y, z = pf.exec_to_paper_mm(T[:3, 3], T[:3, :3])
        xs.append(x); ys.append(y); zs.append(z)
    return np.array(xs), np.array(ys), np.array(zs)


def main():
    cfg = data_io.load_config()
    po = cfg['path_overlay']
    fkc = cfg['fk']

    ap = argparse.ArgumentParser(
        description='Plot 2-D pen-tip tracking error vs time (drawing window).')
    ap.add_argument('csv', help='draw_capture.csv from capture.py')
    ap.add_argument('--start', type=float, default=None,
                    help='trim start (t_rel_s in the CSV, seconds)')
    ap.add_argument('--end', type=float, default=None,
                    help='trim end   (t_rel_s in the CSV, seconds)')
    ap.add_argument('--outdir', default=data_io.figures_dir(cfg))
    ap.add_argument('--out', default='error_vs_time.png',
                    help='output filename (PNG + PDF written side-by-side)')
    ap.add_argument('--no-pen-filter', action='store_true',
                    help='show the whole trimmed window, not just pen-down range')
    ap.add_argument('--show', action='store_true')
    args = ap.parse_args()

    if not args.show:
        matplotlib.use('Agg')

    P = _load_helpers()

    # ── load CSV ──────────────────────────────────────────────────────────────
    if not os.path.exists(args.csv):
        sys.exit(f'ERROR: CSV not found: {args.csv}')
    d = data_io.read_csv(args.csv, fkc['joint_names'])
    m = (np.all(np.isfinite(d['enc']), axis=1) &
         np.all(np.isfinite(d['cmd']), axis=1))
    m &= data_io.time_mask(d['t'], args.start, args.end)
    t  = d['t'][m]
    Q_cmd = d['cmd'][m]
    Q_enc = d['enc'][m]

    if t.size == 0:
        sys.exit('ERROR: no valid rows after masking.')

    # ── build FK + paper frame ────────────────────────────────────────────────
    urdf = data_io.get_urdf(cfg)
    if urdf is None:
        sys.exit('ERROR: no URDF (no arm_bot.urdf cache and xacro failed).')
    chain = fk_chain.build_chain(urdf, fkc['base_link'], fkc['tip_link'])

    begin    = np.array(po['begin_draw_joints'], float)
    pen_axis = np.array(po['pen_axis_local'],    float)
    pf = P.PaperFrame(chain, begin, po['pen_offset_mm'] / 1000.0,
                      pen_axis, po['paper_rotation_deg'], po['paper_mirror_x'])

    # ── FK → paper-plane pen tips ─────────────────────────────────────────────
    print(f'running FK on {t.size} samples …', flush=True)
    cx, cy, cz = _fk_tips_paper(pf, chain, Q_cmd)
    ex, ey, ez = _fk_tips_paper(pf, chain, Q_enc)

    # ── pen-on-paper detection ────────────────────────────────────────────────
    # FK-derived z has a calibration offset from the paper-frame origin (the arm
    # never sits exactly at z=0 in paper coords). Use the 5th-percentile of
    # commanded z as the paper-surface baseline, then a fixed margin to separate
    # pen-on-paper from between-stroke lifts and large travel moves.
    z_base  = float(np.percentile(cz, 5))          # ≈ paper surface z
    pd_margin_mm = 10.0                             # mm above baseline = still "on paper"
    pen_down = cz <= (z_base + pd_margin_mm)        # tight: actual drawing strokes
    # "near paper" = wider band used to define the drawing window boundaries
    near_paper = cz <= (z_base + 60.0)

    # ── registration-offset removal (same correction gen_path_overlay uses) ──
    enc_near = ez <= (float(np.percentile(ez, 5)) + 60.0)
    if pen_down.any() and enc_near.any():
        dx = np.nanmean(ex[enc_near]) - np.nanmean(cx[near_paper])
        dy = np.nanmean(ey[enc_near]) - np.nanmean(cy[near_paper])
        ex -= dx; ey -= dy
        print(f'registration offset removed: ({dx:+.1f}, {dy:+.1f}) mm')

    # ── 2-D pen-tip error at every sample ────────────────────────────────────
    err_mm = np.hypot(cx - ex, cy - ey)  # (N,)

    # ── drawing window: first to last "near paper" sample ────────────────────
    # We clip to the near-paper zone so large pre/post travel is excluded.
    if near_paper.any() and not args.no_pen_filter:
        first_pd = np.argmax(near_paper)
        last_pd  = len(near_paper) - 1 - np.argmax(near_paper[::-1])
        win = slice(first_pd, last_pd + 1)
    else:
        win = slice(None)

    t_win      = t[win]
    err_win    = err_mm[win]
    pd_win     = pen_down[win]           # tight pen-on-paper mask
    np_win     = near_paper[win]         # wide "near paper" mask
    t_draw     = t_win - t_win[0]       # zero at drawing start

    # ── stats (pen-down only) ─────────────────────────────────────────────────
    pd_err = err_win[pd_win]
    if pd_err.size:
        rms_mm = float(np.sqrt(np.mean(pd_err ** 2)))
        max_mm = float(np.max(pd_err))
        print(f'pen-down samples: {pd_err.size}')
        print(f'tracking error:  RMS {rms_mm:.3f} mm,  max {max_mm:.3f} mm')
    else:
        rms_mm = max_mm = np.nan
        print('WARNING: no pen-down samples found — check pen_up threshold or --no-pen-filter')

    # ── figure ────────────────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(10.0, 4.5))

    def _shade_spans(mask, color, alpha):
        """Shade contiguous True runs in mask as axvspan."""
        in_span, span_start = False, None
        for i, v in enumerate(mask):
            if v and not in_span:
                span_start = t_draw[i]; in_span = True
            elif not v and in_span:
                ax.axvspan(span_start, t_draw[i], color=color, alpha=alpha,
                           lw=0, zorder=0)
                in_span = False
        if in_span:
            ax.axvspan(span_start, t_draw[-1], color=color, alpha=alpha, lw=0, zorder=0)

    # light grey background for between-stroke lifts (near paper but pen up)
    lifted = np_win & ~pd_win
    _shade_spans(lifted, '#cccccc', 0.30)

    # draw the full error in faint grey, then overlay pen-on-paper in red
    ax.plot(t_draw, err_win, color='#bbbbbb', lw=0.7, zorder=1)

    def _plot_segments(mask, color, lw, zorder):
        seg_x, seg_y = [], []
        for i, v in enumerate(mask):
            if v:
                seg_x.append(t_draw[i]); seg_y.append(err_win[i])
            else:
                if seg_x:
                    ax.plot(seg_x, seg_y, color=color, lw=lw, zorder=zorder)
                seg_x, seg_y = [], []
        if seg_x:
            ax.plot(seg_x, seg_y, color=color, lw=lw, zorder=zorder)

    _plot_segments(pd_win, '#d62728', 1.4, 3)

    # RMS and max lines
    if np.isfinite(rms_mm):
        ax.axhline(rms_mm, ls='--', color='#1f77b4', lw=1.2,
                   label=f'RMS {rms_mm:.2f} mm', zorder=4)
        ax.axhline(max_mm, ls=':',  color='#ff7f0e', lw=1.2,
                   label=f'max {max_mm:.2f} mm', zorder=4)

    pen_patch  = mpatches.Patch(color='#d62728', label='pen on paper')
    lift_patch = mpatches.Patch(color='#cccccc', alpha=0.7, label='pen lifted (between strokes)')
    travel_patch = mpatches.Patch(color='#bbbbbb', alpha=0.6, label='travel / approach')
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles=handles + [pen_patch, lift_patch, travel_patch],
              loc='upper right', framealpha=0.9, fontsize=9)

    ax.set_xlabel('Time since drawing start (s)', fontsize=11)
    ax.set_ylabel('2-D pen-tip error (mm)', fontsize=11)
    ax.set_xlim(0, t_draw[-1])

    # y-axis: scale to pen-on-paper range; clip large travel spikes so the
    # drawing data fills the frame rather than being squashed at the bottom.
    y_top = max(15.0, max_mm * 1.3) if np.isfinite(max_mm) else 20.0
    ax.set_ylim(0, y_top)
    n_clipped = int(np.sum(err_win > y_top))
    if n_clipped:
        ax.text(0.99, 0.97, f'{n_clipped} travel samples clipped (>{y_top:.0f} mm)',
                transform=ax.transAxes, ha='right', va='top', fontsize=8,
                color='#666666', style='italic')

    ax.grid(True, ls=':', alpha=0.4)

    title = 'Pen-tip tracking error vs time'
    if np.isfinite(rms_mm):
        title += f'  —  RMS {rms_mm:.2f} mm,  max {max_mm:.2f} mm  (pen-on-paper)'
    ax.set_title(title, fontsize=12)

    fig.tight_layout()

    os.makedirs(args.outdir, exist_ok=True)
    stem = args.out if not args.out.endswith('.png') else args.out[:-4]
    png_path = os.path.join(args.outdir, stem + '.png')
    pdf_path = os.path.join(args.outdir, stem + '.pdf')
    fig.savefig(png_path, dpi=300)
    fig.savefig(pdf_path)
    print(f'wrote {png_path}')
    print(f'wrote {pdf_path}')

    if args.show:
        plt.show()


if __name__ == '__main__':
    main()
