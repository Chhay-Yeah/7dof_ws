#!/usr/bin/env python3
"""ee_target_marker.py — cheap RViz debug spheres for the EE target vs actual.

Publishes a single, in-place-updated MarkerArray so you can SEE what the
Cartesian jog is commanding:

  • RED sphere   = commanded /ee_target  (what the GUI is asking for)
  • GREEN sphere = actual /ee_pose       (where the arm really is, from FK)

The gap between them is the tracking error / the "lead carrot". When the jog
pushes the target past the reachable workspace, the red sphere keeps going while
the green one stalls — that divergence is the bug you're hunting.

Why this and not gz_path_tracer: each marker reuses the SAME id, so RViz just
MOVES it every update instead of accumulating geometry. RViz markers are nearly
free, unlike spawning a Gazebo entity per point (which tanks the sim). Add a
"MarkerArray" display in RViz on topic /ee_markers to see them.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class EeTargetMarker(Node):
    def __init__(self) -> None:
        super().__init__("ee_target_marker")
        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("scale", 0.03)          # sphere diameter [m]
        self._base = self.get_parameter("base_link").value
        self._scale = float(self.get_parameter("scale").value)

        self._target: Marker | None = None
        self._actual: Marker | None = None

        self.create_subscription(PoseStamped, "/ee_target", self._cb_target, 10)
        self.create_subscription(PoseStamped, "/ee_pose", self._cb_actual, 10)
        self._pub = self.create_publisher(MarkerArray, "/ee_markers", 10)
        # Republish at a modest fixed rate so RViz always has the latest even if
        # it (re)connects after the last message — still trivially cheap.
        self.create_timer(0.1, self._publish)
        self.get_logger().info(
            "ee_target_marker: red=/ee_target green=/ee_pose -> /ee_markers")

    def _make(self, msg: PoseStamped, mid: int, rgb) -> Marker:
        m = Marker()
        m.header.frame_id = msg.header.frame_id or self._base
        m.ns = "ee_markers"
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = msg.pose
        m.scale.x = m.scale.y = m.scale.z = self._scale
        m.color.r, m.color.g, m.color.b = rgb
        m.color.a = 0.9
        return m

    def _cb_target(self, msg: PoseStamped) -> None:
        self._target = self._make(msg, 0, (1.0, 0.1, 0.1))   # red

    def _cb_actual(self, msg: PoseStamped) -> None:
        self._actual = self._make(msg, 1, (0.1, 0.9, 0.1))   # green

    def _publish(self) -> None:
        arr = MarkerArray()
        if self._target is not None:
            self._target.header.stamp = self.get_clock().now().to_msg()
            arr.markers.append(self._target)
        if self._actual is not None:
            self._actual.header.stamp = self.get_clock().now().to_msg()
            arr.markers.append(self._actual)
        if arr.markers:
            self._pub.publish(arr)


def main() -> None:
    rclpy.init()
    node = EeTargetMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
