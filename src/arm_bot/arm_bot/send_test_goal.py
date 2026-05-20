#!/usr/bin/env python3
"""Send test moveL goals relative to the current EE pose.

Modes:
  single   (default) — fire one moveL with offset (dx, dy, dz)
  sequence           — fire a fixed 6-move benchmark suite covering ±X, ±Y, ±Z
                       returns to home between moves; waits ~`hold` seconds
                       between fires so each motion settles

Use:
    ros2 run arm_bot send_test_goal.py
    ros2 run arm_bot send_test_goal.py --ros-args -p dx:=0.05
    ros2 run arm_bot send_test_goal.py --ros-args -p mode:=sequence
    ros2 run arm_bot send_test_goal.py --ros-args -p mode:=sequence -p hold:=4.0
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped

_DOUBLE = ParameterDescriptor(type=Parameter.Type.DOUBLE.value)


class SendTestGoal(Node):
    def __init__(self):
        super().__init__('send_test_goal')

        self.declare_parameter('dx', 0.0, _DOUBLE)
        self.declare_parameter('dy', 0.05, _DOUBLE)
        self.declare_parameter('dz', 0.0, _DOUBLE)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('timeout_s', 10.0, _DOUBLE)
        self.declare_parameter('mode', 'single')
        self.declare_parameter('hold', 4.0, _DOUBLE)

        self._dx = float(self.get_parameter('dx').value)
        self._dy = float(self.get_parameter('dy').value)
        self._dz = float(self.get_parameter('dz').value)
        self._frame = self.get_parameter('frame_id').value
        self._timeout = float(self.get_parameter('timeout_s').value)
        self._mode = str(self.get_parameter('mode').value)
        self._hold = float(self.get_parameter('hold').value)

        self._cur = None
        self._sub = self.create_subscription(PoseStamped, '/ee_pose', self._on_pose, 5)
        self._pub = self.create_publisher(PoseStamped, '/move_l_goal', 5)

        self.get_logger().info(
            f'waiting for /ee_pose (timeout {self._timeout:.1f}s) '
            f'then sending moveL  d=({self._dx:.3f}, {self._dy:.3f}, {self._dz:.3f}) m'
        )

    def _on_pose(self, msg: PoseStamped):
        if self._cur is None:
            self._cur = msg

    def _build_goal(self, dx, dy, dz) -> PoseStamped:
        # Always relative to the *home* pose captured at startup so each move
        # in a sequence is comparable, not stacked on top of the previous.
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self._frame
        goal.pose.position.x = self._home.pose.position.x + dx
        goal.pose.position.y = self._home.pose.position.y + dy
        goal.pose.position.z = self._home.pose.position.z + dz
        goal.pose.orientation = self._home.pose.orientation
        return goal

    def _fire_one(self, dx, dy, dz, label=""):
        goal = self._build_goal(dx, dy, dz)
        for _ in range(3):
            self._pub.publish(goal)
            time.sleep(0.05)
        self.get_logger().info(
            f'{label} /move_l_goal  d=({dx:+.3f}, {dy:+.3f}, {dz:+.3f}) → '
            f'({goal.pose.position.x:.3f}, {goal.pose.position.y:.3f}, {goal.pose.position.z:.3f})'
        )

    def fire(self) -> bool:
        if self._cur is None:
            self.get_logger().error('no /ee_pose received — is the IK pipeline running?')
            return False
        # Capture the starting pose as "home" for relative offsets.
        self._home = self._cur

        if self._mode == 'single':
            self._fire_one(self._dx, self._dy, self._dz, label='[single]')
            return True

        if self._mode == 'sequence':
            steps = [
                ( 0.05,  0.0,   0.0,  '[+X]'),
                (-0.05,  0.0,   0.0,  '[-X]'),  # back through home
                ( 0.0,   0.05,  0.0,  '[+Y]'),
                ( 0.0,  -0.05,  0.0,  '[-Y]'),
                ( 0.0,   0.0,   0.05, '[+Z]'),
                ( 0.0,   0.0,  -0.05, '[-Z]'),
            ]
            self.get_logger().info(
                f'sequence: {len(steps)} moveLs of ±5 cm, hold={self._hold:.1f}s'
            )
            for dx, dy, dz, lbl in steps:
                self._fire_one(dx, dy, dz, label=lbl)
                time.sleep(self._hold)
            self.get_logger().info('sequence complete')
            return True

        self.get_logger().error(f"unknown mode '{self._mode}' (use 'single' or 'sequence')")
        return False


def main():
    rclpy.init()
    node = SendTestGoal()
    deadline = time.time() + node._timeout
    try:
        while rclpy.ok() and node._cur is None and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        ok = node.fire()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
