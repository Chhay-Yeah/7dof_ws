#!/usr/bin/env python3
"""go_to_start.py — on backend startup, move the arm to a conditioned start pose.

Real hardware powers on wherever the encoders happen to read; you then command
it to a safe operating pose. This mirrors that (and the drawing planner's
move-to-begin): once the controller is up, publish a JointTrajectory that moves
the arm to a well-conditioned, ELBOW-UP start pose — so it never operates from
the all-zeros configuration, which is an exact kinematic singularity (Jacobian
condition ~1e16) that makes Cartesian jog thrash.

It's a joint-space move (no IK), so it can leave the singularity cleanly. Sent a
few times to survive controller-activation timing, then the node goes idle.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = [f"joint_{i}" for i in range(1, 8)]


class GoToStart(Node):
    def __init__(self) -> None:
        super().__init__("go_to_start")
        # Elbow-up, well-conditioned (cond ~18, vs ~34 elbow-down and 1e16 at
        # zeros); matches the region the drawing planner uses successfully.
        self.declare_parameter("start_joints", [0.0, -0.4, 0.0, 1.2, 0.0, 0.0, 0.0])
        self.declare_parameter("move_seconds", 3.0)
        self.declare_parameter("delay_seconds", 2.5)   # let the controller activate
        self.declare_parameter("sends", 3)             # resend count (timing safety)
        self.declare_parameter("controller_topic",
                               "/arm_controller/joint_trajectory")
        self._start = [float(x) for x in self.get_parameter("start_joints").value]
        self._move_s = float(self.get_parameter("move_seconds").value)
        self._delay = float(self.get_parameter("delay_seconds").value)
        self._sends = int(self.get_parameter("sends").value)
        topic = self.get_parameter("controller_topic").value

        self._pub = self.create_publisher(JointTrajectory, topic, 10)
        self._armed = False
        self._count = 0
        self._timer = None
        self.create_subscription(JointState, "/joint_states", self._cb, 10)
        self.get_logger().info(
            f"go_to_start: will move to {self._start} once the controller is up")

    def _cb(self, _msg: JointState) -> None:
        # First sign of life from the stack — wait `delay` for the controller to
        # activate, then publish (periodic timer; we cancel after `sends`).
        if self._armed:
            return
        self._armed = True
        self._timer = self.create_timer(self._delay, self._tick)

    def _tick(self) -> None:
        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = list(self._start)
        sec = int(self._move_s)
        pt.time_from_start = Duration(sec=sec,
                                      nanosec=int((self._move_s - sec) * 1e9))
        traj.points.append(pt)
        self._pub.publish(traj)
        self._count += 1
        if self._count == 1:
            self.get_logger().info(f"go_to_start: moving to {self._start}")
        if self._count >= self._sends and self._timer is not None:
            self._timer.cancel()
            self.get_logger().info("go_to_start: done")


def main() -> None:
    rclpy.init()
    node = GoToStart()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
