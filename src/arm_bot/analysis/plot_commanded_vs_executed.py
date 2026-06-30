#!/usr/bin/env python3
"""
plot_commanded_vs_executed.py  —  Figure 4.1 generator.

Overlays, in the drawing plane, the path the batch planner COMMANDED against the
path the arm actually EXECUTED during a drawing run.

  • commanded path  = the resampled Cartesian waypoints the batch planner emitted,
                      read from the one-shot /cartesian_path PoseArray it publishes
                      per drawing (these are EE targets in base frame).
  • executed path   = the end-effector position obtained by running forward
                      kinematics on every recorded /joint_states sample.

Both are converted to PEN-TIP coordinates in the paper plane (mm, canvas-centred)
using exactly the same transform the planner uses internally (see
DrawingBatchPlanner._publish_pen_pos), so the two curves live in one frame and
the gap between them is the real tracking error.

────────────────────────────────────────────────────────────────────────────────
RECORD A RUN (do this once, with the pendant backend + GUI already up):

    # source the workspace first
    source /opt/ros/humble/setup.bash
    source ~/7dof_ws/install/setup.bash

    # start recording BEFORE you click "Send" in the Drawing tab, so the bag
    # catches the one-shot /cartesian_path message:
    ros2 bag record -o draw_run /joint_states /cartesian_path /robot_description

    # → draw in the pendant, let the whole motion finish (approach, draw, lift),
    #   then Ctrl-C the bag.

MAKE THE FIGURE:

    python3 src/arm_bot/analysis/plot_commanded_vs_executed.py draw_run --show

    # If /robot_description didn't make it into the bag, pass a URDF instead:
    #   ros2 topic echo --once --field data /robot_description > /tmp/rd.urdf
    #   python3 .../plot_commanded_vs_executed.py draw_run --urdf /tmp/rd.urdf

Defaults for begin_draw_joints / pen / paper rotation match
pendant_backend.launch.py. If you ran draw_and_execute_batch.launch.py or tuned
the planner, pass the matching --begin-draw-joints / --paper-rotation-deg etc.
────────────────────────────────────────────────────────────────────────────────
"""
import argparse
import os
import sys

import numpy as np
import matplotlib
import matplotlib.pyplot as plt


# ── URDF-driven FK chain ─────────────────────────────────────────────────────
# Inlined verbatim from arm_bot/fk_arm_v3.py so this script is self-contained:
# it needs only a stock ROS install (/opt/ros) + the URDF from the bag, NOT the
# arm_bot workspace overlay on PYTHONPATH. The FK is therefore identical to the
# live fk_arm_v3 node that produced /ee_pose.

def _rpy_to_R(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def _origin_T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = _rpy_to_R(*rpy)
    T[:3,  3] = xyz
    return T


def _axis_angle_R(axis, angle):
    a = axis / np.linalg.norm(axis)
    x, y, z = a
    c, s = np.cos(angle), np.sin(angle)
    C = 1 - c
    return np.array([
        [c + x*x*C,   x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s, c + y*y*C,   y*z*C - x*s],
        [z*x*C - y*s, z*y*C + x*s, c + z*z*C],
    ])


class UrdfChain:
    """Serial-chain FK built from a URDF (copy of fk_arm_v3.UrdfChain)."""

    def __init__(self, urdf_xml, base, tip):
        from urdf_parser_py.urdf import URDF
        robot = URDF.from_xml_string(urdf_xml)
        parent_of = {j.child: (j, j.parent) for j in robot.joints}
        chain = []
        link = tip
        while link != base:
            if link not in parent_of:
                raise RuntimeError(f"link '{link}' has no parent (chain to '{base}' broken)")
            joint, parent = parent_of[link]
            chain.append(joint)
            link = parent
        chain.reverse()
        self.joints = []
        self.joint_names = []
        for j in chain:
            xyz = list(j.origin.xyz) if j.origin and j.origin.xyz else [0, 0, 0]
            rpy = list(j.origin.rpy) if j.origin and j.origin.rpy else [0, 0, 0]
            axis = np.array(j.axis if j.axis is not None else [0, 0, 1], dtype=float)
            self.joints.append({"name": j.name, "type": j.type,
                                "T_origin": _origin_T(xyz, rpy), "axis": axis})
            if j.type in ("revolute", "continuous"):
                self.joint_names.append(j.name)
        self.n = len(self.joint_names)

    def fk(self, q):
        T = np.eye(4)
        qi = 0
        for j in self.joints:
            T = T @ j["T_origin"]
            if j["type"] in ("revolute", "continuous"):
                Rh = np.eye(4)
                Rh[:3, :3] = _axis_angle_R(j["axis"], q[qi])
                T = T @ Rh
                qi += 1
        return T


# ── Geometry helpers ─────────────────────────────────────────────────────────

def quat_xyzw_to_R(qx, qy, qz, qw):
    """geometry_msgs quaternion (x, y, z, w) → 3×3 rotation matrix."""
    q = np.array([qx, qy, qz, qw], dtype=float)
    n = np.linalg.norm(q)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = q / n
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def paper_basis(rotation_deg, mirror_x):
    """The base→paper rotation the planner builds (about base +Z)."""
    th = np.radians(rotation_deg)
    c, s = float(np.cos(th)), float(np.sin(th))
    sx = -1.0 if mirror_x else 1.0
    return np.array([
        [c * sx, -s, 0.0],
        [s * sx,  c, 0.0],
        [0.0,     0.0, 1.0],
    ])


class PaperFrame:
    """Bridges the two native frames the data arrives in, so commanded and
    executed end up on the same axes.

      • The planner publishes /cartesian_path already in PAPER coordinates
        (canvas-centred metres; the PoseArray positions ARE the waypoint
        (x,y,z) tuples), so the commanded path needs no transform — just ×1000.
      • Executed comes from FK as a base-frame EE pose, so we map its pen tip
        INTO the paper frame exactly as DrawingBatchPlanner._publish_pen_pos
        does: paper origin = pen tip at begin_draw, axes = paper_R (rotation
        about base +Z). Because the wrist orientation is held during drawing,
        the constant pen offset cancels and executed pen-tip-paper coincides
        with the planner's EE waypoint frame.

    With frame='base' both are expressed in base-frame mm instead (executed =
    raw EE position; commanded = paper waypoint mapped to base via begin_draw),
    a sanity check that doesn't lean on the paper recentring.
    """

    def __init__(self, chain, begin_draw_joints, pen_offset_m, pen_axis_local,
                 rotation_deg, mirror_x):
        self.pen_offset_m = pen_offset_m
        self.pen_axis_local = pen_axis_local / np.linalg.norm(pen_axis_local)
        q_b = np.minimum(np.maximum(np.asarray(begin_draw_joints, float),
                                    chain_q_min(chain)), chain_q_max(chain))
        T_b = chain.fk(q_b)
        self.T_begin = T_b
        self.R_begin = T_b[:3, :3]
        self.paper_R = paper_basis(rotation_deg, mirror_x)
        self.anchor = T_b[:3, 3] + pen_offset_m * (self.R_begin @ self.pen_axis_local)

    def pen_tip(self, p_base, R_base):
        return p_base + self.pen_offset_m * (R_base @ self.pen_axis_local)

    def exec_to_paper_mm(self, p_base, R_base):
        """Base-frame EE pose → pen-tip position in paper mm (x, y, z)."""
        paper = self.paper_R.T @ (self.pen_tip(p_base, R_base) - self.anchor)
        return paper * 1000.0

    def cmd_paper_to_base_mm(self, p_paper_m):
        """Paper-frame waypoint (m) → base-frame EE position in mm (--frame base)."""
        return (self.T_begin[:3, 3] + self.paper_R @ np.asarray(p_paper_m)) * 1000.0


def chain_q_min(chain):
    # fk_arm_v3.UrdfChain doesn't expose limits; default to no clamp if absent.
    return getattr(chain, 'q_min', np.full(chain.n, -np.inf))


def chain_q_max(chain):
    return getattr(chain, 'q_max', np.full(chain.n, np.inf))


# ── Pen-up line breaking ─────────────────────────────────────────────────────

def break_on_pen_up(x, y, z, up_thresh):
    """Insert NaNs where the pen is lifted so a plotted line lifts off the
    paper instead of drawing connecting segments across travel/approach moves."""
    x = np.asarray(x, float).copy()
    y = np.asarray(y, float).copy()
    up = np.asarray(z, float) > up_thresh
    x[up] = np.nan
    y[up] = np.nan
    return x, y


def auto_pen_up_thresh(z_cmd):
    """Pick a pen-up height between the paper (min z) and the lift height."""
    z = np.asarray(z_cmd, float)
    if z.size == 0:
        return np.inf
    zmin, zmax = float(np.min(z)), float(np.max(z))
    return zmin + max(1.0, 0.3 * (zmax - zmin))


# ── Tracking error ───────────────────────────────────────────────────────────

def point_to_polyline_mm(px, py, poly_x, poly_y):
    """Min distance from point (px,py) to the commanded polyline, in plot units."""
    ax = poly_x[:-1]; ay = poly_y[:-1]
    bx = poly_x[1:];  by = poly_y[1:]
    vx = bx - ax;     vy = by - ay
    wx = px - ax;     wy = py - ay
    seg2 = vx*vx + vy*vy
    t = np.divide(wx*vx + wy*vy, seg2, out=np.zeros_like(seg2),
                  where=seg2 > 1e-12)
    t = np.clip(t, 0.0, 1.0)
    cx = ax + t*vx; cy = ay + t*vy
    d = np.hypot(px - cx, py - cy)
    return float(np.min(d)) if d.size else np.nan


def tracking_error(exe_x, exe_y, cmd_x, cmd_y):
    """RMS / max nearest-distance from executed pen samples to the commanded
    polyline. NaN-broken (pen-up) points are skipped on both sides."""
    cm = np.isfinite(cmd_x) & np.isfinite(cmd_y)
    cx, cy = cmd_x[cm], cmd_y[cm]
    if cx.size < 2:
        return np.nan, np.nan, 0
    dists = []
    for ex, ey in zip(exe_x, exe_y):
        if not (np.isfinite(ex) and np.isfinite(ey)):
            continue
        dists.append(point_to_polyline_mm(ex, ey, cx, cy))
    if not dists:
        return np.nan, np.nan, 0
    d = np.asarray(dists)
    return float(np.sqrt(np.mean(d*d))), float(np.max(d)), len(d)


# ── Bag reading ──────────────────────────────────────────────────────────────

def detect_storage_id(uri):
    if os.path.isdir(uri):
        files = os.listdir(uri)
        if any(f.endswith('.mcap') for f in files):
            return 'mcap'
        if any(f.endswith('.db3') for f in files):
            return 'sqlite3'
    return 'sqlite3'


def read_bag(uri):
    """Returns (urdf_xml_or_None, [PoseArray...], [(t_sec, JointState)...])."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=uri, storage_id=detect_storage_id(uri))
    conv = rosbag2_py.ConverterOptions('', '')
    reader.open(storage, conv)
    typemap = {t.name: t.type for t in reader.get_all_topics_and_types()}

    urdf = None
    cart = []
    joints = []
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic not in typemap:
            continue
        msg = deserialize_message(data, get_message(typemap[topic]))
        if topic == '/robot_description':
            urdf = msg.data
        elif topic == '/cartesian_path':
            cart.append(msg)
        elif topic == '/joint_states':
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            joints.append((ts, msg))
    return urdf, cart, joints


# ── Figure ───────────────────────────────────────────────────────────────────

def make_figure(cmd_x, cmd_y, exe_x, exe_y, out_path, frame_label,
                rms=None, mx=None, show=False, title='Figure 4.1 — Commanded vs Executed Path'):
    fig, ax = plt.subplots(figsize=(6.0, 6.0))
    ax.plot(cmd_x, cmd_y, '-', color='#1f77b4', lw=2.0, label='commanded', zorder=2)
    ax.plot(exe_x, exe_y, '-', color='#d62728', lw=1.2, alpha=0.85,
            label='measured', zorder=3)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, ls=':', alpha=0.5)
    ax.set_xlabel(f'{frame_label} X (mm)')
    ax.set_ylabel(f'{frame_label} Y (mm)')
    ax.set_title(title)
    # Add 20 % margin so the legend never sits on top of the path
    x_all = np.concatenate([cmd_x[np.isfinite(cmd_x)], exe_x[np.isfinite(exe_x)]])
    y_all = np.concatenate([cmd_y[np.isfinite(cmd_y)], exe_y[np.isfinite(exe_y)]])
    if x_all.size and y_all.size:
        pad = max(5.0, 0.20 * max(x_all.max() - x_all.min(), y_all.max() - y_all.min()))
        cx_ = (x_all.min() + x_all.max()) / 2
        cy_ = (y_all.min() + y_all.max()) / 2
        half = max(x_all.max() - x_all.min(), y_all.max() - y_all.min()) / 2 + pad
        ax.set_xlim(cx_ - half, cx_ + half)
        ax.set_ylim(cy_ - half, cy_ + half)
    ax.legend(loc='lower right', framealpha=0.9)
    fig.tight_layout()
    # Encode error metrics in the filename so they're readable without opening the image
    cap = ''
    if rms is not None and np.isfinite(rms):
        cap = f'RMS{rms:.2f}mm_max{mx:.2f}mm'
        stem, ext = os.path.splitext(out_path)
        out_path = f'{stem}_{cap}{ext}'
    fig.savefig(out_path, dpi=300)
    print(f'wrote {out_path}')
    if out_path.lower().endswith('.png'):
        pdf = out_path[:-4] + '.pdf'
        fig.savefig(pdf)
        print(f'wrote {pdf}')
    if show:
        plt.show()


# ── Self-test (no bag): proves the plot/transform/error path runs ────────────

def selftest(args):
    th = np.linspace(0, 2*np.pi, 240)
    r = 15.0
    cmd_x = r*np.cos(th); cmd_y = r*np.sin(th)
    cmd_z = np.zeros_like(th)
    # executed = commanded + slow drift + noise + a small lag
    rng_x = cmd_x + 0.6*np.sin(3*th) + 0.3*np.cos(th)
    rng_y = cmd_y + 0.6*np.cos(3*th) - 0.3*np.sin(th)
    up = auto_pen_up_thresh(cmd_z)
    cx, cy = break_on_pen_up(cmd_x, cmd_y, cmd_z, up)
    ex, ey = break_on_pen_up(rng_x, rng_y, np.zeros_like(th), up)
    rms, mx, n = tracking_error(ex, ey, cx, cy)
    make_figure(cx, cy, ex, ey, args.out, 'paper', rms, mx, show=args.show,
                title='Figure 4.1 (SELF-TEST synthetic data)')
    print(f'selftest OK — {n} executed samples scored')


# ── Main ─────────────────────────────────────────────────────────────────────

def build_chain(urdf_xml, base_link, tip_link):
    return UrdfChain(urdf_xml, base_link, tip_link)


def joint_q(msg, joint_names, cache={}):
    key = tuple(msg.name)
    idx = cache.get(key)
    if idx is None:
        idx = [msg.name.index(n) for n in joint_names]
        cache[key] = idx
    return np.array([msg.position[i] for i in idx], dtype=float)


def main():
    ap = argparse.ArgumentParser(description='Figure 4.1 — commanded vs executed drawing path overlay.')
    ap.add_argument('bag', nargs='?', help='rosbag2 directory with /joint_states, /cartesian_path, /robot_description')
    ap.add_argument('--out', default='figure_4_1.png', help='output image path (also writes a .pdf next to a .png)')
    ap.add_argument('--urdf', help='URDF file, used if /robot_description is not in the bag')
    ap.add_argument('--frame', choices=['paper', 'base'], default='paper',
                    help="'paper' = canvas-centred drawing plane (default); 'base' = raw base-frame XY")
    ap.add_argument('--base-link', default='base_link')
    ap.add_argument('--tip-link', default='ee')
    # paper-plane / pen params — defaults match pendant_backend.launch.py
    ap.add_argument('--begin-draw-joints', default='0.0,-0.7,0.0,1.4,0.01,0.0,1.0')
    ap.add_argument('--pen-offset-mm', type=float, default=100.0)
    ap.add_argument('--pen-axis-local', default='1,0,0')
    ap.add_argument('--paper-rotation-deg', type=float, default=270.0)
    ap.add_argument('--paper-mirror-x', action='store_true')
    # executed-window timing (drops the move-to-begin swing + dwell)
    ap.add_argument('--move-to-begin', type=float, default=4.0)
    ap.add_argument('--dwell', type=float, default=3.0)
    ap.add_argument('--settle', type=float, default=0.5)
    ap.add_argument('--approach', type=float, default=1.0,
                    help='approach_seconds: extra lead skipped so the move from '
                         'begin-pose out to the first stroke point is trimmed')
    ap.add_argument('--no-time-window', action='store_true',
                    help='plot the whole executed bag (incl. approach swing), not just the draw window')
    ap.add_argument('--pen-up-mm', type=float, default=None,
                    help='pen-up break height in mm (default: auto from commanded z range)')
    ap.add_argument('--show', action='store_true')
    ap.add_argument('--selftest', action='store_true',
                    help='generate a synthetic figure with no bag (sanity check)')
    args = ap.parse_args()

    matplotlib.use('Agg') if not args.show else None

    if args.selftest:
        selftest(args)
        return

    if not args.bag:
        ap.error('a bag directory is required (or use --selftest)')

    begin = np.array([float(v) for v in args.begin_draw_joints.split(',')])
    pen_axis = np.array([float(v) for v in args.pen_axis_local.split(',')])
    pen_off_m = args.pen_offset_mm / 1000.0

    print(f'reading bag: {args.bag}')
    urdf, cart_msgs, joint_msgs = read_bag(args.bag)
    if urdf is None and args.urdf:
        with open(args.urdf) as f:
            urdf = f.read()
    if urdf is None:
        sys.exit('ERROR: no /robot_description in the bag and no --urdf given. '
                 'Re-record with /robot_description, or pass --urdf <file>.')
    if not cart_msgs:
        sys.exit('ERROR: no /cartesian_path in the bag — start `ros2 bag record` '
                 'BEFORE clicking Send so it catches the one-shot commanded path.')
    if not joint_msgs:
        sys.exit('ERROR: no /joint_states in the bag.')

    chain = build_chain(urdf, args.base_link, args.tip_link)
    print(f'URDF chain: {chain.n} DoF — {chain.joint_names}')

    pf = PaperFrame(chain, begin, pen_off_m, pen_axis,
                    args.paper_rotation_deg, args.paper_mirror_x)
    paper_frame = (args.frame == 'paper')

    # ── commanded path (last PoseArray = the actual drawing planned) ─────────
    # /cartesian_path positions are already PAPER-frame metres (the planner's
    # waypoint tuples), so in paper frame they just scale to mm; in base frame
    # we map them through begin_draw.
    pa = cart_msgs[-1]
    cmd = np.array([
        ([ps.position.x * 1000.0, ps.position.y * 1000.0, ps.position.z * 1000.0]
         if paper_frame else
         pf.cmd_paper_to_base_mm([ps.position.x, ps.position.y, ps.position.z]))
        for ps in pa.poses
    ])
    cmd_x, cmd_y, cmd_z = cmd[:, 0], cmd[:, 1], cmd[:, 2]
    t0 = pa.header.stamp.sec + pa.header.stamp.nanosec * 1e-9

    # ── executed path (FK of every joint sample) ────────────────────────────
    joint_msgs.sort(key=lambda kv: kv[0])
    draw_start = t0 + args.move_to_begin + args.dwell + args.settle + args.approach
    ex, ey, ez = [], [], []
    skipped = 0
    for ts, msg in joint_msgs:
        if not args.no_time_window and ts < draw_start:
            continue
        try:
            q = joint_q(msg, chain.joint_names)
        except ValueError:
            skipped += 1
            continue
        T = chain.fk(q)
        if paper_frame:
            x, y, z = pf.exec_to_paper_mm(T[:3, 3], T[:3, :3])
        else:
            x, y, z = T[0, 3] * 1000.0, T[1, 3] * 1000.0, T[2, 3] * 1000.0
        ex.append(x); ey.append(y); ez.append(z)
    ex, ey, ez = np.array(ex), np.array(ey), np.array(ez)
    if skipped:
        print(f'warning: {skipped} joint samples missing chain joints (skipped)')
    if ex.size == 0:
        sys.exit('ERROR: no executed samples in the draw window. Try '
                 '--no-time-window, or adjust --move-to-begin/--dwell.')

    print(f'commanded waypoints: {cmd_x.size};  executed samples: {ex.size}')

    # ── pen-up breaks so multi-stroke lifts read as separate strokes ────────
    up = args.pen_up_mm if args.pen_up_mm is not None else auto_pen_up_thresh(cmd_z)
    cx, cy = break_on_pen_up(cmd_x, cmd_y, cmd_z, up)
    fx, fy = break_on_pen_up(ex, ey, ez, up)

    rms, mx, n = tracking_error(fx, fy, cx, cy)
    if np.isfinite(rms):
        print(f'tracking error over {n} pen-down samples: RMS {rms:.2f} mm, max {mx:.2f} mm')

    make_figure(cx, cy, fx, fy, args.out,
                'paper' if args.frame == 'paper' else 'base',
                rms, mx, show=args.show)


if __name__ == '__main__':
    main()
