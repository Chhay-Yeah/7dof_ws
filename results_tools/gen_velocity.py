#!/usr/bin/env python3
"""
gen_velocity.py — Deliverable 2: seven joint-velocity traces vs time.

A PyQt6 window (matches the pendant stack) plotting joint_1..joint_7 velocity,
one tab10-coloured trace each, styled like the existing make_thesis_figures.py
Figure 4.4. Runs in two modes:

  --live            subscribe to /joint_states during a draw, rolling window
  --replay BAG      load a recorded run and plot the whole velocity profile

An Export button (or --export) writes the SAME plot to PNG + PDF — this doubles
as thesis Figure 4.5 (figures/figure_4_5.{png,pdf}).

VELOCITY SOURCE: there is no motor-reported velocity on ROS (see README,
Phase 0), so velocity is the numeric derivative of the encoder position with a
light Savitzky-Golay filter. The path taken is printed in the plot subtitle.

    source ~/7dof_ws/install/setup.bash
    python3 results_tools/gen_velocity.py --replay draw_run
    python3 results_tools/gen_velocity.py --replay draw_run --export   # headless
    python3 results_tools/gen_velocity.py --live
"""
import argparse
import os
import sys
import time
from collections import deque

import numpy as np

import data_io

JOINT_COLORS = None  # filled after matplotlib import


# ── shared styling (matches make_thesis_figures.fig_joint_velocities) ────────

def style_axes(ax, names):
    ax.set_xlabel('time (s)')
    ax.set_ylabel('joint velocity (rad/s)')
    ax.grid(True, ls=':', alpha=0.5)


def draw_traces(ax, t, vel, names, label):
    ax.clear()
    for k, name in enumerate(names):
        ax.plot(t, vel[:, k], lw=1.3, color=JOINT_COLORS[k], label=name)
    ax.set_title(f'Joint velocity profiles ({label})')
    style_axes(ax, names)
    ax.legend(ncol=4, fontsize=8, loc='upper right', framealpha=0.9)
    if len(t) >= 2:
        ax.set_xlim(float(np.min(t)), float(np.max(t)))


def export(fig, outdir, stem):
    os.makedirs(outdir, exist_ok=True)
    png = os.path.join(outdir, stem + '.png')
    pdf = os.path.join(outdir, stem + '.pdf')
    fig.tight_layout()
    fig.savefig(png, dpi=300)
    fig.savefig(pdf)
    print(f'wrote {png} + {pdf}')


# ── headless replay export ───────────────────────────────────────────────────

def replay_series(source, cfg, force_diff=False, lo=None, hi=None):
    """Return (t_rel, vel[N,7], names, label) from a recorded bag OR an encoder
    CSV (auto-detected). For a CSV the default is the MEASURED encoder velocity
    (real motor velocity); --diff differentiates the measured position instead.
    lo/hi (t_rel seconds) trim to a window — e.g. just the drawing portion."""
    names = cfg['fk']['joint_names']

    if data_io.is_csv(source):
        d = data_io.read_csv(source, names)
        t = d['t']
        if not force_diff and np.isfinite(d['enc_vel']).any():
            m = np.all(np.isfinite(d['enc_vel']), axis=1)
            t_out, vel, label = t[m], d['enc_vel'][m], 'measured encoder velocity'
        else:
            m = np.all(np.isfinite(d['enc']), axis=1)
            tm, pos = t[m], d['enc'][m]
            grid = data_io.make_timeline(tm.min(), tm.max(), cfg['timeline']['resample_fps'])
            pos_g = data_io.resample(tm, pos, grid)
            vel, lab = data_io.velocity_from_position(grid, pos_g, cfg)
            t_out, label = grid, 'measured encoder pos, ' + lab
    else:
        data = data_io.read_bag(source)
        if not data['joint_states']:
            sys.exit('ERROR: no /joint_states in bag.')
        t, pos = data_io.jointstate_series(data['joint_states'], names)
        trel = t - data_io.time_origin(data, cfg)
        grid = data_io.make_timeline(trel.min(), trel.max(), cfg['timeline']['resample_fps'])
        pos_g = data_io.resample(trel, pos, grid)
        vel, label = data_io.velocity_from_position(grid, pos_g, cfg)
        t_out = grid

    if lo is not None or hi is not None:
        w = data_io.time_mask(t_out, lo, hi)
        t_out, vel = t_out[w], vel[w]
        label += f' [{lo if lo is not None else t_out.min():.1f}–{hi if hi is not None else t_out.max():.1f}s]'
    return t_out, vel, names, label


# ── live ROS subscriber (decoupled — subscribe only) ─────────────────────────

class LiveVel:
    """Buffers /joint_states and yields a rolling (t, vel) window. Pure
    observer: subscribes, never publishes."""

    def __init__(self, cfg):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        self.cfg = cfg
        self.names = cfg['fk']['joint_names']
        self.window = float(cfg['velocity'].get('live_window_s', 12.0))
        self.t = deque()
        self.pos = deque()
        self._t0 = None

        rclpy.init()
        self.node = Node('results_tools_live_velocity')
        topic = cfg['record'].get('feedback_topic', '/joint_states')
        self._idx = None
        self.node.create_subscription(JointState, topic, self._cb, 50)
        self.node.get_logger().info(f'observing {topic} (subscribe-only)')

    def _cb(self, msg):
        if self._idx is None or self._idx_key != tuple(msg.name):
            try:
                self._idx = [msg.name.index(n) for n in self.names]
                self._idx_key = tuple(msg.name)
            except ValueError:
                return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = stamp if stamp > 0 else time.monotonic()
        if self._t0 is None:
            self._t0 = now
        self.t.append(now - self._t0)
        self.pos.append([msg.position[i] for i in self._idx])
        while self.t and (self.t[-1] - self.t[0]) > self.window:
            self.t.popleft(); self.pos.popleft()

    def spin_once(self):
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def current(self):
        if len(self.t) < 6:
            return None
        t = np.asarray(self.t)
        pos = np.asarray(self.pos)
        order = np.argsort(t)
        t, pos = t[order], pos[order]
        vel, label = data_io.velocity_from_position(t, pos, self.cfg)
        return t, vel, label

    def shutdown(self):
        import rclpy
        self.node.destroy_node()
        rclpy.shutdown()


# ── PyQt6 window ─────────────────────────────────────────────────────────────

def run_gui(cfg, live=False, bag=None, force_diff=False, lo=None, hi=None):
    os.environ.setdefault('QT_API', 'pyqt6')
    import matplotlib
    matplotlib.use('qtagg')
    global JOINT_COLORS
    JOINT_COLORS = matplotlib.colormaps['tab10'].colors
    from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.figure import Figure
    from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                                 QHBoxLayout, QPushButton, QLabel)
    from PyQt6.QtCore import QTimer

    names = cfg['fk']['joint_names']
    app = QApplication(sys.argv)
    win = QMainWindow()
    win.setWindowTitle('Joint velocities — ' + ('LIVE' if live else 'replay'))
    central = QWidget(); win.setCentralWidget(central)
    layout = QVBoxLayout(central)

    fig = Figure(figsize=(7.5, 4.5))
    ax = fig.add_subplot(111)
    canvas = FigureCanvas(fig)
    layout.addWidget(canvas)

    row = QHBoxLayout()
    status = QLabel('')
    btn = QPushButton('Export PNG + PDF (Fig 4.5)')
    row.addWidget(status); row.addStretch(1); row.addWidget(btn)
    layout.addLayout(row)

    state = {'t': None, 'vel': None, 'label': '—'}

    def do_export():
        if state['vel'] is None:
            status.setText('nothing to export yet'); return
        export(fig, data_io.figures_dir(cfg), cfg['velocity']['export_stem'])
        status.setText(f"exported {cfg['velocity']['export_stem']}.png/.pdf")
    btn.clicked.connect(do_export)

    if live:
        src = LiveVel(cfg)

        def tick():
            src.spin_once()
            cur = src.current()
            if cur is None:
                status.setText('waiting for /joint_states…'); return
            t, vel, label = cur
            state.update(t=t, vel=vel, label=label)
            draw_traces(ax, t, vel, names, label + ' — live')
            canvas.draw_idle()
            status.setText(f'{len(t)} samples · {label}')

        timer = QTimer(); timer.timeout.connect(tick); timer.start(50)
        win._timer = timer; win._src = src
    else:
        t, vel, names_, label = replay_series(bag, cfg, force_diff=force_diff, lo=lo, hi=hi)
        state.update(t=t, vel=vel, label=label)
        draw_traces(ax, t, vel, names, label)
        canvas.draw_idle()
        pk = np.max(np.abs(vel), axis=0)
        status.setText('peak |v|: ' + ', '.join(f'{n}={v:.2f}' for n, v in zip(names, pk)))

    win.resize(900, 600)
    win.show()
    rc = app.exec()
    if live:
        win._src.shutdown()
    sys.exit(rc)


def main():
    cfg = data_io.load_config()
    ap = argparse.ArgumentParser(description='Joint velocity figure — live or replay.')
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--live', action='store_true', help='subscribe to /joint_states')
    g.add_argument('--replay', metavar='BAG_OR_CSV',
                   help='plot a recorded rosbag2 dir OR an encoder .csv (auto-detected)')
    ap.add_argument('--export', action='store_true',
                    help='headless: write the velocity figure and exit (replay only)')
    ap.add_argument('--outdir', default=None, help='override output dir for --export')
    ap.add_argument('--diff', action='store_true',
                    help='CSV mode: differentiate measured position instead of using '
                         'the measured encoder velocity')
    ap.add_argument('--start', type=float, default=None,
                    help='trim: window start in t_rel seconds (e.g. just the drawing)')
    ap.add_argument('--end', type=float, default=None, help='trim: window end (t_rel s)')
    args = ap.parse_args()

    if args.export:
        if not args.replay:
            sys.exit('--export requires --replay BAG')
        import matplotlib
        matplotlib.use('Agg')
        global JOINT_COLORS
        JOINT_COLORS = matplotlib.colormaps['tab10'].colors
        from matplotlib.figure import Figure
        t, vel, names, label = replay_series(args.replay, cfg, force_diff=args.diff,
                                             lo=args.start, hi=args.end)
        fig = Figure(figsize=(7.5, 4.5)); ax = fig.add_subplot(111)
        draw_traces(ax, t, vel, names, label)
        outdir = args.outdir or data_io.figures_dir(cfg)
        export(fig, outdir, cfg['velocity']['export_stem'])
        pk = np.max(np.abs(vel), axis=0)
        print('peak |velocity| per joint (rad/s): ' +
              ', '.join(f'{n}={v:.3f}' for n, v in zip(names, pk)))
        return

    run_gui(cfg, live=args.live, bag=args.replay, force_diff=args.diff,
            lo=args.start, hi=args.end)


if __name__ == '__main__':
    main()
