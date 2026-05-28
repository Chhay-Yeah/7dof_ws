#!/usr/bin/env python3
"""
drawing_executor_node.py

Subscribes:  /cartesian_path  (geometry_msgs/PoseArray)
Publishes:   /ee_target       (geometry_msgs/PoseStamped)

Streams a Cartesian path to the live IK node (ik_arm_v3) one small, uniform
step at a time so the IK tracks at a constant speed.

The incoming PoseArray from drawing_trajectory_planner mixes wildly uneven
spacing: mouse-drawn stroke points sit sub-mm apart, while the pen-down hop
(~30 mm vertical) and inter-stroke travel (100+ mm) are single giant pose
steps. Relaying those verbatim at a fixed rate makes the EE lurch — the IK's
per-step joint cap means it is still slewing toward a far target when the next
one overwrites it, causing overshoot. We resample the path into segments no
longer than `draw_speed_mm_s / rate_hz` in position (orientation SLERPed across
each segment) and emit one sub-waypoint per tick, giving a steady Cartesian
speed of `draw_speed_mm_s`.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Pose


def _slerp(q0, q1, t):
    """SLERP between quaternions (w, x, y, z), shortest arc."""
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1, dot = -q1, -dot
    if dot > 0.9995:
        r = q0 + t * (q1 - q0)
        return r / np.linalg.norm(r)
    theta_0 = np.arccos(dot)
    sin_t0 = np.sin(theta_0)
    theta = theta_0 * t
    s0 = np.cos(theta) - dot * np.sin(theta) / sin_t0
    s1 = np.sin(theta) / sin_t0
    return s0 * q0 + s1 * q1


class ExecutorNode(Node):
    def __init__(self):
        super().__init__('drawing_executor_node')

        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('draw_speed_mm_s', 15.0)
        self.declare_parameter('frame_id', 'base_link')

        self._rate = max(1.0, float(self.get_parameter('rate_hz').value))
        v = max(1e-3, float(self.get_parameter('draw_speed_mm_s').value))
        self._frame = str(self.get_parameter('frame_id').value)
        # Distance covered per published step. Constant speed = step * rate.
        self._step_m = (v / self._rate) / 1000.0

        self._queue: list = []  # list of (pos (3,), quat_wxyz (4,))

        self.create_subscription(
            PoseArray, '/cartesian_path', self.on_path, 10)
        self._pub = self.create_publisher(PoseStamped, '/ee_target', 10)

        self._timer = self.create_timer(1.0 / self._rate, self.tick)
        self.get_logger().info(
            f'Executor ready — streaming /ee_target at {self._rate:.1f} Hz, '
            f'{self._step_m * 1000:.2f} mm/step (~{v:.0f} mm/s)')

    def on_path(self, msg: PoseArray):
        if msg.header.frame_id:
            self._frame = msg.header.frame_id
        raw = [self._pose_to_pq(p) for p in msg.poses]
        self._queue = self._resample(raw)
        self.get_logger().info(
            f'Received path: {len(msg.poses)} poses → {len(self._queue)} '
            f'steps (frame={self._frame})')

    @staticmethod
    def _pose_to_pq(p: Pose):
        pos = np.array([p.position.x, p.position.y, p.position.z], dtype=float)
        quat = np.array([p.orientation.w, p.orientation.x,
                         p.orientation.y, p.orientation.z], dtype=float)
        return pos, quat

    def _resample(self, raw: list) -> list:
        """Subdivide each segment so no step exceeds `self._step_m`,
        linearly interpolating position and SLERPing orientation."""
        if len(raw) < 2:
            return list(raw)
        out = [raw[0]]
        for (p0, q0), (p1, q1) in zip(raw[:-1], raw[1:]):
            dist = float(np.linalg.norm(p1 - p0))
            n = max(1, int(np.ceil(dist / self._step_m)))
            for i in range(1, n + 1):
                t = i / n
                out.append((p0 + (p1 - p0) * t, _slerp(q0, q1, t)))
        return out

    def tick(self):
        if not self._queue:
            return

        pos, quat = self._queue.pop(0)
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame
        out.pose.position.x = float(pos[0])
        out.pose.position.y = float(pos[1])
        out.pose.position.z = float(pos[2])
        out.pose.orientation.w = float(quat[0])
        out.pose.orientation.x = float(quat[1])
        out.pose.orientation.y = float(quat[2])
        out.pose.orientation.z = float(quat[3])
        self._pub.publish(out)

        self.get_logger().info(
            f'→ /ee_target: ({pos[0]:+.3f}, {pos[1]:+.3f}, '
            f'{pos[2]:+.3f})  remaining={len(self._queue)}',
            throttle_duration_sec=0.5,
        )


def main():
    rclpy.init()
    rclpy.spin(ExecutorNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
