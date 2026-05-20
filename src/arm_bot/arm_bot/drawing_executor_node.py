#!/usr/bin/env python3
"""
drawing_executor_node.py

Subscribes:  /cartesian_path  (geometry_msgs/PoseArray)
Publishes:   /ee_target       (geometry_msgs/PoseStamped)

Buffers an incoming PoseArray and streams one PoseStamped at a time on
/ee_target at a fixed rate, pacing the IK node (ik_arm_v3) through the
drawing path.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped


class ExecutorNode(Node):
    def __init__(self):
        super().__init__('drawing_executor_node')

        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('frame_id', 'base_link')
        self._rate = float(self.get_parameter('rate_hz').value)
        self._frame = str(self.get_parameter('frame_id').value)

        self._queue: list = []

        self.create_subscription(
            PoseArray, '/cartesian_path', self.on_path, 10)
        self._pub = self.create_publisher(PoseStamped, '/ee_target', 10)

        self._timer = self.create_timer(1.0 / self._rate, self.tick)
        self.get_logger().info(
            f'Executor ready — streaming /ee_target at {self._rate:.1f} Hz')

    def on_path(self, msg: PoseArray):
        self._queue = list(msg.poses)
        if msg.header.frame_id:
            self._frame = msg.header.frame_id
        self.get_logger().info(
            f'Received path: {len(self._queue)} poses (frame={self._frame})')

    def tick(self):
        if not self._queue:
            return

        pose = self._queue.pop(0)
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame
        out.pose = pose
        self._pub.publish(out)

        self.get_logger().info(
            f'→ /ee_target: ({pose.position.x:+.3f}, {pose.position.y:+.3f}, '
            f'{pose.position.z:+.3f})  remaining={len(self._queue)}',
            throttle_duration_sec=0.5,
        )


def main():
    rclpy.init()
    rclpy.spin(ExecutorNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
