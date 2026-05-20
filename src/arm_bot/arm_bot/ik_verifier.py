#!/usr/bin/env python3
"""IK tracking-error monitor.

Subscribes to /ee_target (commanded pose) and /ee_pose (FK output of the
actual joint state). Publishes the position-error (mm) and orientation-error
(deg) on /ee_tracking_error so it can be plotted live with rqt_plot.

Logs a one-line summary every few seconds with min/max/avg error since last
print, so you have a quantitative quality signal for any IK/Cartesian work.

Usage:
    ros2 run arm_bot ik_verifier.py
    ros2 run rqt_plot rqt_plot \
        /ee_tracking_error/position_error_mm \
        /ee_tracking_error/orientation_error_deg
"""

import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Float32, Header
from rcl_interfaces.msg import ParameterDescriptor


def quat_angle(q1, q2):
    """Smallest rotation angle (rad) between two quaternions [x,y,z,w]."""
    d = abs(float(np.dot(q1, q2)))
    d = max(-1.0, min(1.0, d))
    return 2.0 * math.acos(d)


class IKError(Node):
    """Custom message-free output: publish individual scalar topics so rqt_plot
    can chart them directly. Also publish a combined Vector3 with x=pos_mm,
    y=ori_deg, z=age_ms for one-topic plotting if you prefer."""

    def __init__(self):
        super().__init__('ik_verifier')

        self.declare_parameter('summary_period', 5.0,
                               ParameterDescriptor(description='Seconds between log summaries'))

        self._period = float(self.get_parameter('summary_period').value)

        # Latest of each — pair on every new sample. Avoids sim-time vs wall-time
        # stamp mismatch that breaks naive timestamp matching.
        self._target = None     # (stamp_sec, pos[3], quat[4])
        self._pose = None
        self._samples = deque(maxlen=2000)   # (pos_err_m, ori_err_rad)
        self._last_log = time.time()

        self.create_subscription(PoseStamped, '/ee_target', self._on_target, 50)
        self.create_subscription(PoseStamped, '/ee_pose',   self._on_pose,   50)

        self._pos_pub  = self.create_publisher(Float32, '/ee_tracking_error/position_error_mm',    20)
        self._ori_pub  = self.create_publisher(Float32, '/ee_tracking_error/orientation_error_deg', 20)
        self._vec_pub  = self.create_publisher(Vector3, '/ee_tracking_error/vector', 20)
        # Latency between target and matched pose (ms)
        self._lag_pub  = self.create_publisher(Float32, '/ee_tracking_error/lag_ms', 20)

        self.create_timer(self._period, self._log_summary)
        self.get_logger().info(
            f'ik_verifier ready  pairing latest /ee_target with latest /ee_pose  '
            f'summary every {self._period:.1f} s'
        )

    @staticmethod
    def _ps_to_arrays(msg: PoseStamped):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
        q = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w], dtype=float)
        n = np.linalg.norm(q)
        if n < 1e-9:
            q = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            q = q / n
        return t, p, q

    def _on_target(self, msg: PoseStamped):
        self._target = self._ps_to_arrays(msg)
        self._try_pair()

    def _on_pose(self, msg: PoseStamped):
        self._pose = self._ps_to_arrays(msg)
        self._try_pair()

    def _try_pair(self):
        if self._target is None or self._pose is None:
            return
        t_t, p_t, q_t = self._target
        t_p, p_p, q_p = self._pose
        lag = abs(t_t - t_p)   # informational only

        pos_err = float(np.linalg.norm(p_t - p_p))
        ori_err = quat_angle(q_t, q_p)

        self._samples.append((pos_err, ori_err))

        self._pos_pub.publish(Float32(data=float(pos_err * 1000.0)))
        self._ori_pub.publish(Float32(data=float(math.degrees(ori_err))))
        self._lag_pub.publish(Float32(data=float(lag * 1000.0)))
        v = Vector3()
        v.x = float(pos_err * 1000.0)
        v.y = float(math.degrees(ori_err))
        v.z = float(lag * 1000.0)
        self._vec_pub.publish(v)

    def _log_summary(self):
        if not self._samples:
            self.get_logger().info('no paired target/pose samples yet '
                                   '(check /ee_target and /ee_pose are publishing)')
            return
        arr = np.array(self._samples)
        pos_mm  = arr[:, 0] * 1000.0
        ori_deg = np.degrees(arr[:, 1])
        self.get_logger().info(
            f'tracking error  pos: avg={pos_mm.mean():.2f}  max={pos_mm.max():.2f} mm   '
            f'ori: avg={ori_deg.mean():.3f}  max={ori_deg.max():.3f} deg   '
            f'(N={len(arr)})'
        )
        self._samples.clear()


def main():
    rclpy.init()
    node = IKError()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
