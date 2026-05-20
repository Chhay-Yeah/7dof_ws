#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointRelay(Node):
    def __init__(self):
        super().__init__("joint_relay")

        # ---- parameters ----
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("joint_names", [
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"
        ])
        self.declare_parameter("initial_positions", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.joint_names = list(self.get_parameter("joint_names").value)
        init_pos = list(self.get_parameter("initial_positions").value)

        if len(self.joint_names) != 7:
            self.get_logger().warn("joint_names should have 7 items. Using first 7.")
            self.joint_names = (self.joint_names + [f"joint_{i+1}" for i in range(7)])[:7]

        if len(init_pos) != 7:
            self.get_logger().warn("initial_positions should have 7 items. Using zeros.")
            init_pos = [0.0] * 7

        # last commanded state (what we publish)
        self.q = np.array(init_pos, dtype=float)
        self.qdot = np.zeros(7, dtype=float)

        # ROS
        self.sub = self.create_subscription(JointState, "/joint_commands", self.cb_cmd, 10)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_loop)

        self.get_logger().info(
            f"JointRelay publishing /joint_states at {self.rate_hz} Hz with initial pose."
        )

    def cb_cmd(self, msg: JointState):
        # Map incoming command by name to our fixed joint order
        if not msg.name or len(msg.name) == 0:
            # If no names, assume same order
            if len(msg.position) >= 7:
                self.q = np.array(msg.position[:7], dtype=float)
            if msg.velocity and len(msg.velocity) >= 7:
                self.qdot = np.array(msg.velocity[:7], dtype=float)
            return

        name_to_idx = {n: i for i, n in enumerate(msg.name)}

        # Update positions
        if msg.position and len(msg.position) > 0:
            for j, jn in enumerate(self.joint_names):
                if jn in name_to_idx and name_to_idx[jn] < len(msg.position):
                    self.q[j] = float(msg.position[name_to_idx[jn]])

        # Update velocities (optional)
        if msg.velocity and len(msg.velocity) > 0:
            for j, jn in enumerate(self.joint_names):
                if jn in name_to_idx and name_to_idx[jn] < len(msg.velocity):
                    self.qdot[j] = float(msg.velocity[name_to_idx[jn]])

    def publish_loop(self):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = self.joint_names
        out.position = self.q.tolist()
        out.velocity = self.qdot.tolist()
        # effort can be left empty
        self.pub.publish(out)


def main():
    rclpy.init()
    node = JointRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()