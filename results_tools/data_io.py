#!/usr/bin/env python3
"""
data_io.py — recording I/O + time alignment for results_tools.

Reads a rosbag2 recording (sqlite3 or mcap), pulls out every stream the three
generators need, and resamples each onto a SHARED uniform timeline built from
the bag's own message stamps — so the velocity figure, the path overlay, the
front-view animation, and your video all agree on t.

Units are SI throughout: radians, metres, seconds. Joint vectors are always in
the canonical order passed in (indexed BY NAME for /joint_states, by array
position for the bare /motor_target Float64MultiArray).
"""
import os
import numpy as np
import yaml

# Workspace root = parent of results_tools/, so relative output paths in
# config.yaml (e.g. figures_dir: figures) anchor to ~/7dof_ws/figures
# regardless of the current working directory.
WS_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ── config ───────────────────────────────────────────────────────────────────

def load_config(path=None):
    if path is None:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config.yaml')
    with open(path) as f:
        return yaml.safe_load(f)


def figures_dir(cfg):
    """Resolve io.figures_dir against the workspace root if it is relative."""
    d = cfg['io']['figures_dir']
    return d if os.path.isabs(d) else os.path.join(WS_ROOT, d)


# ── URDF resolution (bag has none in CSV mode) ───────────────────────────────

_HERE = os.path.dirname(os.path.abspath(__file__))


def get_urdf(cfg, bag_urdf=None):
    """URDF string for FK. Prefers (1) a bag's /robot_description, (2) the
    config fk.urdf_path, (3) the cached results_tools/arm_bot.urdf, (4) a live
    xacro expansion. CSV recordings carry no URDF, so this is how they get FK."""
    if bag_urdf:
        return bag_urdf
    p = cfg['fk'].get('urdf_path')
    if p:
        p = os.path.expanduser(p)
        if not os.path.isabs(p):
            p = os.path.join(WS_ROOT, p)
        if os.path.exists(p):
            return open(p).read()
    cached = os.path.join(_HERE, 'arm_bot.urdf')
    if os.path.exists(cached):
        return open(cached).read()
    import subprocess
    xacro = os.path.join(WS_ROOT, 'src/arm_bot/urdf/arm_bot.urdf.xacro')
    return subprocess.check_output(['xacro', xacro], text=True)


# ── CSV recording (from record_encoder.py) ───────────────────────────────────

def read_csv(path, names):
    """Read an encoder CSV (record_encoder.py). Returns a dict:
        t        : (N,)   draw-relative time (s)
        cmd      : (N, J) commanded joint position (rad)
        enc      : (N, J) MEASURED encoder position (rad)
        enc_vel  : (N, J) MEASURED encoder velocity (rad/s)
    indexed in the canonical joint order `names`."""
    import csv as _csv
    t, cmd, enc, vel = [], [], [], []
    with open(path) as f:
        r = _csv.DictReader(f)
        for row in r:
            t.append(float(row['t_rel_s']))
            cmd.append([float(row[f'{j}_cmd_rad']) for j in names])
            enc.append([float(row[f'{j}_enc_rad']) for j in names])
            vel.append([float(row[f'{j}_enc_vel_radps']) for j in names])
    return {'t': np.asarray(t, float), 'cmd': np.asarray(cmd, float),
            'enc': np.asarray(enc, float), 'enc_vel': np.asarray(vel, float)}


def is_csv(path):
    return isinstance(path, str) and path.lower().endswith('.csv')


# ── bag reading (generalized — all streams) ──────────────────────────────────

def _detect_storage_id(uri):
    if os.path.isdir(uri):
        files = os.listdir(uri)
        if any(f.endswith('.mcap') for f in files):
            return 'mcap'
        if any(f.endswith('.db3') for f in files):
            return 'sqlite3'
    return 'sqlite3'


def read_bag(uri):
    """Read a rosbag2 directory. Returns a dict:
        urdf            : str | None       (/robot_description)
        joint_states    : [(t, JointState)]
        joint_commands  : [(t, JointState)]
        cartesian_path  : [PoseArray]
        motor_target    : [(t, [float]*N)] (/motor_target Float64MultiArray)
    t is the message stamp (header.stamp) when present, else the bag receive time.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=uri, storage_id=_detect_storage_id(uri))
    reader.open(storage, rosbag2_py.ConverterOptions('', ''))
    typemap = {t.name: t.type for t in reader.get_all_topics_and_types()}

    out = {'urdf': None, 'joint_states': [], 'joint_commands': [],
           'cartesian_path': [], 'motor_target': []}

    while reader.has_next():
        topic, data, t_recv = reader.read_next()
        typ = typemap.get(topic)
        if typ is None:
            continue
        msg = deserialize_message(data, get_message(typ))
        t_recv_s = t_recv * 1e-9
        if topic == '/robot_description':
            out['urdf'] = msg.data
        elif topic == '/cartesian_path':
            out['cartesian_path'].append(msg)
        elif topic == '/joint_states':
            out['joint_states'].append((_stamp(msg, t_recv_s), msg))
        elif topic == '/joint_commands':
            out['joint_commands'].append((_stamp(msg, t_recv_s), msg))
        elif topic == '/motor_target':
            out['motor_target'].append((t_recv_s, list(msg.data)))
    return out


def _stamp(msg, fallback):
    try:
        s = msg.header.stamp
        v = s.sec + s.nanosec * 1e-9
        return v if v > 0.0 else fallback
    except AttributeError:
        return fallback


# ── joint extraction (canonical order) ───────────────────────────────────────

def jointstate_series(joint_msgs, names):
    """[(t, JointState)] -> (t[N], pos[N, J]) indexed BY NAME, sorted by t."""
    idx_cache = {}
    t, pos = [], []
    for ts, msg in joint_msgs:
        key = tuple(msg.name)
        ii = idx_cache.get(key)
        if ii is None:
            ii = [msg.name.index(n) for n in names]
            idx_cache[key] = ii
        t.append(ts)
        pos.append([msg.position[i] for i in ii])
    t = np.asarray(t, float)
    pos = np.asarray(pos, float)
    order = np.argsort(t)
    return t[order], pos[order]


def motortarget_series(motor_msgs, names):
    """[(t, [float])] -> (t[M], pos[M, J]). The bare Float64MultiArray has no
    names, so array position == canonical joint order (joint_1..J)."""
    J = len(names)
    t, pos = [], []
    for ts, arr in motor_msgs:
        if len(arr) < J:
            continue
        t.append(ts)
        pos.append(arr[:J])
    t = np.asarray(t, float)
    pos = np.asarray(pos, float)
    if t.size == 0:
        return t, pos.reshape(0, J)
    order = np.argsort(t)
    return t[order], pos[order]


# ── shared timeline + resampling ─────────────────────────────────────────────

def time_origin(data, cfg):
    """t=0 reference (s). Trajectory dispatch stamp if configured & present."""
    if cfg['timeline'].get('zero_at_cartesian_path', True) and data['cartesian_path']:
        pa = data['cartesian_path'][-1]
        return pa.header.stamp.sec + pa.header.stamp.nanosec * 1e-9
    if data['joint_states']:
        return min(t for t, _ in data['joint_states'])
    return 0.0


def make_timeline(t_lo, t_hi, fps):
    n = max(2, int(round((t_hi - t_lo) * fps)) + 1)
    return np.linspace(t_lo, t_hi, n)


def resample(t_src, arr_src, t_grid):
    """Linear-interpolate columns of arr_src(t_src) onto t_grid. Handles the
    unsorted / duplicate-stamp case. arr_src is (N,) or (N, K)."""
    arr_src = np.asarray(arr_src, float)
    if arr_src.ndim == 1:
        return np.interp(t_grid, t_src, arr_src)
    out = np.empty((len(t_grid), arr_src.shape[1]))
    for k in range(arr_src.shape[1]):
        out[:, k] = np.interp(t_grid, t_src, arr_src[:, k])
    return out


# ── velocity: differentiate position + light filter ──────────────────────────

def velocity_from_position(t, pos, cfg):
    """Numeric velocity (rad/s) from joint positions, with a light filter.

    No real motor-reported velocity exists on ROS (Phase 0), so velocity is the
    central difference of the encoder position, lightly smoothed. Returns
    (vel[N, J], label) where label names the path taken (for the plot subtitle).
    """
    vc = cfg['velocity']
    pos = np.asarray(pos, float)
    filt = vc.get('filter', 'savgol')

    if filt == 'savgol':
        from scipy.signal import savgol_filter
        win = int(vc.get('savgol_window', 21))
        win = min(win, _odd_le(len(t)))
        if win >= 5:
            poly = min(int(vc.get('savgol_polyorder', 3)), win - 1)
            dt = float(np.median(np.diff(t))) if len(t) > 1 else 1.0
            # derivative directly from the Savitzky-Golay fit (uniform grid)
            vel = savgol_filter(pos, win, poly, deriv=1, delta=dt, axis=0)
            return vel, f'numeric diff, Savitzky-Golay (win={win}, poly={poly})'
        # too few samples — fall through to plain gradient

    vel = np.gradient(pos, t, axis=0)
    if filt == 'ema':
        a = float(vc.get('ema_alpha', 0.25))
        vel = _ema(vel, a)
        return vel, f'numeric diff, EMA (alpha={a})'
    return vel, 'numeric diff (unfiltered)'


def _odd_le(n):
    return n if n % 2 == 1 else n - 1


def _ema(x, a):
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = a * x[i] + (1 - a) * y[i - 1]
    return y
