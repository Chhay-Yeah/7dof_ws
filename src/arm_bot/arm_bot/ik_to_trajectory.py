#!/usr/bin/env python3
"""
ik_to_trajectory.py — bridge /joint_commands (sensor_msgs/JointState)
to /arm_controller/joint_trajectory (trajectory_msgs/JointTrajectory).

Each incoming JointState is reordered into the canonical 7-joint list
expected by the JointTrajectoryController and forwarded as a single-point
trajectory with time_from_start = step_horizon_s.
"""
from builtin_interfaces.msg import Duration

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


CANONICAL_JOINTS = [
    "joint_1", "joint_2", "joint_3", "joint_4",
    "joint_5", "joint_6", "joint_7",
]


class IKToTrajectory(Node):
    def __init__(self):
        super().__init__("ik_to_trajectory")

        self.declare_parameter("step_horizon_s", 0.08)
        self._horizon = float(self.get_parameter("step_horizon_s").value)

        # Cache last full joint vector so partial commands can be merged in
        self._last_q = [0.0] * 7

        self._pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.create_subscription(
            JointState, "/joint_states", self._cb_state, 20
        )
        self.create_subscription(
            JointState, "/joint_commands", self._cb_cmd, 20
        )
        self.get_logger().info(
            f"ik_to_trajectory: horizon={self._horizon*1000:.0f} ms — "
            f"target /arm_controller/joint_trajectory"
        )

    def _cb_state(self, msg: JointState):
        # Track current pose so partial /joint_commands can fill in unspecified joints
        if not msg.name or not msg.position:
            return
        for jn, q in zip(msg.name, msg.position):
            if jn in CANONICAL_JOINTS:
                self._last_q[CANONICAL_JOINTS.index(jn)] = float(q)

    def _cb_cmd(self, msg: JointState):
        if not msg.position:
            self.get_logger().warn(
                "Received /joint_commands with no positions",
                throttle_duration_sec=2.0,
            )
            return

        # Build canonical 7-vector. Use incoming names if provided; otherwise
        # assume the message is already in canonical order.
        q = list(self._last_q)
        if msg.name:
            for jn, p in zip(msg.name, msg.position):
                if jn in CANONICAL_JOINTS:
                    q[CANONICAL_JOINTS.index(jn)] = float(p)
        else:
            for i in range(min(7, len(msg.position))):
                q[i] = float(msg.position[i])

        traj = JointTrajectory()
        traj.joint_names = list(CANONICAL_JOINTS)

        pt = JointTrajectoryPoint()
        pt.positions = q
        sec = int(self._horizon)
        nsec = int((self._horizon - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)

        traj.points.append(pt)
        self._pub.publish(traj)

        # Cache so subsequent partial commands stay consistent
        self._last_q = q

        self.get_logger().info(
            f"→ traj: [{', '.join(f'{v:+.3f}' for v in q)}]",
            throttle_duration_sec=1.0,
        )


def main():
    rclpy.init()
    rclpy.spin(IKToTrajectory())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
