#!/usr/bin/env python3
"""Cartesian path generator: moveL (linear) and moveC (circular arc).

Feeds an interpolated /ee_target stream into the existing IK chain.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from scipy.spatial.transform import Rotation as R, Slerp


def pose_to_arrays(p: Pose):
    pos = np.array([p.position.x, p.position.y, p.position.z], dtype=float)
    quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w], dtype=float)
    n = np.linalg.norm(quat)
    if n < 1e-9:
        quat = np.array([0.0, 0.0, 0.0, 1.0])
    else:
        quat = quat / n
    return pos, quat


def arrays_to_pose(pos, quat) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(pos[0]), float(pos[1]), float(pos[2])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = \
        float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
    return p


def slerp_quat(q0, q1, s):
    key_rot = R.from_quat(np.stack([q0, q1]))
    slerp = Slerp([0.0, 1.0], key_rot)
    return slerp([s]).as_quat()[0]


def angle_between(q0, q1):
    d = abs(float(np.dot(q0, q1)))
    d = min(1.0, max(-1.0, d))
    return 2.0 * math.acos(d)


def circle_from_three_points(p1, p2, p3):
    """Return (center, radius, u_axis, v_axis, theta_end) for arc p1->p2->p3.
    u_axis points from center to p1; v_axis is perpendicular in the circle plane,
    chosen so p2 has positive v component. theta_end is the signed angle to p3.
    """
    v12 = p2 - p1
    v13 = p3 - p1
    n = np.cross(v12, v13)
    n_norm = np.linalg.norm(n)
    if n_norm < 1e-9:
        raise ValueError("Three points are collinear")
    n = n / n_norm

    # Solve in the plane. Use the well-known formula for circumcenter of triangle.
    a = p1
    b = p2
    c = p3
    ab = b - a
    ac = c - a
    d1 = np.dot(ab, ab)
    d2 = np.dot(ac, ac)
    cross_ab_ac = np.cross(ab, ac)
    denom = 2.0 * np.dot(cross_ab_ac, cross_ab_ac)
    if denom < 1e-18:
        raise ValueError("Degenerate circle")
    alpha = (d1 * np.dot(ac, ac) - d2 * np.dot(ab, ac)) / denom
    beta = (d2 * np.dot(ab, ab) - d1 * np.dot(ab, ac)) / denom
    center = a + alpha * ab + beta * ac
    radius = float(np.linalg.norm(p1 - center))

    u = (p1 - center) / radius
    # v perpendicular to u in the plane (n)
    v = np.cross(n, u)
    v = v / np.linalg.norm(v)

    def angle_of(p):
        d = p - center
        x = float(np.dot(d, u))
        y = float(np.dot(d, v))
        return math.atan2(y, x)

    theta2 = angle_of(p2)
    theta3 = angle_of(p3)
    # Ensure direction goes through p2: if theta2 and theta3 have wrong sign relation,
    # flip v so the sweep p1->p2->p3 is in increasing theta.
    if theta2 < 0:
        v = -v
        theta2 = -theta2
        theta3 = -theta3
    # Unwrap theta3 to be >= theta2
    while theta3 < theta2 - 1e-9:
        theta3 += 2.0 * math.pi
    return center, radius, u, v, theta3


class CartesianPath(Node):
    def __init__(self):
        super().__init__('cartesian_path')

        self.declare_parameter('max_lin_vel', 0.05)   # m/s
        self.declare_parameter('max_ang_vel', 0.5)    # rad/s
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('frame_id', 'base_link')

        self.v_lin = float(self.get_parameter('max_lin_vel').value)
        self.v_ang = float(self.get_parameter('max_ang_vel').value)
        self.rate = float(self.get_parameter('publish_rate').value)
        self.frame = self.get_parameter('frame_id').value

        self._cur_pos = None
        self._cur_quat = None

        # Active trajectory state
        self._traj = None  # dict with type + params + t0 + duration

        self.create_subscription(PoseStamped, '/ee_pose', self._on_pose, 20)
        self.create_subscription(PoseStamped, '/move_l_goal', self._on_move_l, 5)
        self.create_subscription(PoseArray, '/move_c_goal', self._on_move_c, 5)

        self._pub = self.create_publisher(PoseStamped, '/ee_target', 10)
        self.create_timer(1.0 / self.rate, self._tick)

        self.get_logger().info(
            f'cartesian_path ready  v_lin={self.v_lin} m/s  v_ang={self.v_ang} rad/s  rate={self.rate} Hz'
        )

    def _on_pose(self, msg: PoseStamped):
        self._cur_pos, self._cur_quat = pose_to_arrays(msg.pose)

    def _duration(self, lin_dist, ang_dist):
        t_lin = lin_dist / self.v_lin if self.v_lin > 0 else 0.0
        t_ang = ang_dist / self.v_ang if self.v_ang > 0 else 0.0
        return max(t_lin, t_ang, 0.05)

    def _on_move_l(self, msg: PoseStamped):
        if self._cur_pos is None:
            self.get_logger().warn('move_l ignored: /ee_pose not yet received')
            return
        end_pos, end_quat = pose_to_arrays(msg.pose)
        start_pos = self._cur_pos.copy()
        start_quat = self._cur_quat.copy()
        lin = float(np.linalg.norm(end_pos - start_pos))
        ang = angle_between(start_quat, end_quat)
        dur = self._duration(lin, ang)
        self._traj = {
            'type': 'L',
            't0': self.get_clock().now().nanoseconds * 1e-9,
            'dur': dur,
            'p0': start_pos, 'p1': end_pos,
            'q0': start_quat, 'q1': end_quat,
        }
        self.get_logger().info(f'moveL  d={lin*1000:.1f} mm  rot={math.degrees(ang):.1f} deg  T={dur:.2f} s')

    def _on_move_c(self, msg: PoseArray):
        if self._cur_pos is None:
            self.get_logger().warn('move_c ignored: /ee_pose not yet received')
            return
        if len(msg.poses) != 2:
            self.get_logger().warn(f'move_c needs exactly 2 poses (via, end); got {len(msg.poses)}')
            return
        via_pos, _ = pose_to_arrays(msg.poses[0])
        end_pos, end_quat = pose_to_arrays(msg.poses[1])
        start_pos = self._cur_pos.copy()
        start_quat = self._cur_quat.copy()
        try:
            center, radius, u_axis, v_axis, theta_end = circle_from_three_points(start_pos, via_pos, end_pos)
        except ValueError as e:
            self.get_logger().warn(f'move_c rejected: {e}')
            return
        arc_len = radius * theta_end
        ang = angle_between(start_quat, end_quat)
        dur = self._duration(arc_len, ang)
        self._traj = {
            'type': 'C',
            't0': self.get_clock().now().nanoseconds * 1e-9,
            'dur': dur,
            'center': center, 'radius': radius,
            'u': u_axis, 'v': v_axis, 'theta_end': theta_end,
            'q0': start_quat, 'q1': end_quat,
        }
        self.get_logger().info(
            f'moveC  r={radius*1000:.1f} mm  arc={math.degrees(theta_end):.1f} deg  '
            f'len={arc_len*1000:.1f} mm  T={dur:.2f} s'
        )

    def _tick(self):
        if self._traj is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        s = (now - self._traj['t0']) / self._traj['dur']
        if s >= 1.0:
            s = 1.0
        if self._traj['type'] == 'L':
            pos = (1.0 - s) * self._traj['p0'] + s * self._traj['p1']
        else:  # 'C'
            theta = s * self._traj['theta_end']
            pos = (self._traj['center']
                   + self._traj['radius'] * (math.cos(theta) * self._traj['u']
                                             + math.sin(theta) * self._traj['v']))
        quat = slerp_quat(self._traj['q0'], self._traj['q1'], s)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.pose = arrays_to_pose(pos, quat)
        self._pub.publish(msg)

        if s >= 1.0:
            self.get_logger().info(f'{self._traj["type"]} done')
            self._traj = None


def main():
    rclpy.init()
    node = CartesianPath()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
