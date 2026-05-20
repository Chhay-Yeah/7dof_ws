#!/usr/bin/env python3
"""Compare two ik_verifier rosbag recordings.

Reads /ee_tracking_error/vector from each bag (Vector3: x=pos_mm, y=ori_deg,
z=lag_ms), prints per-bag and side-by-side stats: avg / p50 / p95 / max.

Use:
    ros2 run arm_bot compare_runs.py /tmp/run_baseline /tmp/run_lambda_001
    ros2 run arm_bot compare_runs.py /tmp/run_a /tmp/run_b --label-a "KDL" --label-b "TRAC-IK"
"""

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as e:
    print(f"missing rosbag2/rclpy modules: {e}", file=sys.stderr)
    sys.exit(2)


def read_vector_topic(bag_dir: str, topic='/ee_tracking_error/vector'):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id='sqlite3'),
                ConverterOptions('', ''))

    type_map = {ti.name: ti.type for ti in reader.get_all_topics_and_types()}
    if topic not in type_map:
        raise RuntimeError(f"topic {topic!r} not in {bag_dir} "
                           f"(found: {list(type_map.keys())})")

    msg_class = get_message(type_map[topic])
    pos_mm, ori_deg, lag_ms = [], [], []
    while reader.has_next():
        t, raw, _stamp = reader.read_next()
        if t != topic:
            continue
        m = deserialize_message(raw, msg_class)
        pos_mm.append(m.x)
        ori_deg.append(m.y)
        lag_ms.append(m.z)
    return np.array(pos_mm), np.array(ori_deg), np.array(lag_ms)


def stats(arr: np.ndarray) -> dict:
    if arr.size == 0:
        return {'n': 0, 'avg': float('nan'), 'p50': float('nan'),
                'p95': float('nan'), 'max': float('nan')}
    return {
        'n': int(arr.size),
        'avg': float(np.mean(arr)),
        'p50': float(np.percentile(arr, 50)),
        'p95': float(np.percentile(arr, 95)),
        'max': float(np.max(arr)),
    }


def fmt_row(label, s):
    return (f"  {label:>5}   N={s['n']:<5}  avg={s['avg']:7.3f}  "
            f"p50={s['p50']:7.3f}  p95={s['p95']:7.3f}  max={s['max']:7.3f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_a')
    ap.add_argument('bag_b')
    ap.add_argument('--label-a', default='A')
    ap.add_argument('--label-b', default='B')
    args = ap.parse_args()

    for b in (args.bag_a, args.bag_b):
        if not Path(b).exists():
            print(f"bag not found: {b}", file=sys.stderr)
            sys.exit(1)

    a_pos, a_ori, a_lag = read_vector_topic(args.bag_a)
    b_pos, b_ori, b_lag = read_vector_topic(args.bag_b)

    for label, pos, ori, lag in [
        (args.label_a, a_pos, a_ori, a_lag),
        (args.label_b, b_pos, b_ori, b_lag),
    ]:
        print(f"\n=== {label} ===")
        print(fmt_row("pos_mm",  stats(pos)))
        print(fmt_row("ori_deg", stats(ori)))
        print(fmt_row("lag_ms",  stats(lag)))

    print(f"\n=== {args.label_a}  →  {args.label_b}  Δ ===")
    a_s, b_s = stats(a_pos), stats(b_pos)
    if a_s['n'] and b_s['n']:
        for key in ('avg', 'p95', 'max'):
            d = b_s[key] - a_s[key]
            pct = (d / a_s[key] * 100.0) if a_s[key] else float('nan')
            arrow = '↑' if d > 0 else '↓'
            print(f"  pos_mm  {key:>3}: {a_s[key]:7.3f} → {b_s[key]:7.3f}  "
                  f"Δ={d:+.3f} ({arrow}{abs(pct):.1f}%)")


if __name__ == '__main__':
    main()
