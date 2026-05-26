#!/usr/bin/env python3
"""
ik_arm_v3.py  —  7-DOF IK node, Damped Least Squares with null-space centering.

FK / Jacobian are now built directly from the URDF on /robot_description, so
the kinematic model matches what robot_state_publisher/RViz visualise exactly
(no DH idealisation, no per-joint axis-sign assumptions).

ROS interface:
  SUB  /joint_states      sensor_msgs/JointState     — current joint angles
  SUB  /ee_target         geometry_msgs/PoseStamped  — desired EE pose
  SUB  /robot_description std_msgs/String (latched)  — URDF
  PUB  /joint_commands    sensor_msgs/JointState     — IK solution
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from arm_bot.ik_lib import UrdfChain, quat_to_rot, rot_error


class IKNode(Node):
    RATE_HZ      = 50.0
    LAMBDA_MAX   = 0.05      # damping when far from goal (numerical safety)
    LAMBDA_MIN   = 5e-4      # damping when close (lets residual shrink to ~µm)
    LAMBDA_KNEE  = 0.05      # error norm at which damping starts to shrink
    POS_GAIN     = 1.0
    ROT_GAIN     = 1.0
    DQ_MAX       = 0.10       # per-step joint-velocity cap [rad]
    INNER_ITERS  = 4          # DLS sub-iterations per timer tick
    TOL_POS      = 1e-5       # 10 µm
    TOL_ROT      = 1e-4       # ~0.006°
    NULL_K       = 0.3

    def __init__(self):
        super().__init__("ik_7dof_v3")

        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("tip_link",  "ee")
        self._base = self.get_parameter("base_link").value
        self._tip  = self.get_parameter("tip_link").value

        self._chain: UrdfChain | None = None
        self._joint_index = None

        self._q      = None
        self._names_in = None
        self._T_des  = None
        self._active = False

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(String, "/robot_description", self._cb_urdf, latched)
        self.create_subscription(JointState,  "/joint_states", self._cb_joints, 30)
        self.create_subscription(PoseStamped, "/ee_target",    self._cb_target, 10)
        self._pub = self.create_publisher(JointState, "/joint_commands", 20)
        self.create_timer(1.0 / self.RATE_HZ, self._step)

        self.get_logger().info(
            f"ik_arm_v3 ready — waiting for /robot_description (chain {self._base} → {self._tip})"
        )

    def _cb_urdf(self, msg: String):
        if self._chain is not None:
            return
        try:
            self._chain = UrdfChain(msg.data, self._base, self._tip)
        except Exception as e:
            self.get_logger().error(f"URDF parse failed: {e}")
            return
        self.get_logger().info(
            f"URDF loaded: {self._chain.n} DoF — joints: {self._chain.joint_names}"
        )

    def _cb_joints(self, msg: JointState):
        if self._chain is None:
            return
        if self._joint_index is None:
            try:
                self._joint_index = [msg.name.index(n) for n in self._chain.joint_names]
            except ValueError as e:
                self.get_logger().warn(
                    f"/joint_states missing one of {self._chain.joint_names}: {e}"
                )
                return
        self._q = np.array([msg.position[i] for i in self._joint_index], dtype=float)
        self._names_in = list(self._chain.joint_names)

    def _cb_target(self, msg: PoseStamped):
        o = msg.pose.orientation
        T = np.eye(4)
        T[:3, :3] = quat_to_rot(o.x, o.y, o.z, o.w)
        T[:3,  3] = [msg.pose.position.x,
                     msg.pose.position.y,
                     msg.pose.position.z]
        self._T_des  = T
        self._active = True

    def _step(self):
        if self._chain is None or self._q is None or not self._active:
            return

        n = self._chain.n
        I6 = np.eye(6)
        In = np.eye(n)

        for _ in range(self.INNER_ITERS):
            J, T_cur = self._chain.jacobian(self._q)
            p_ee = T_cur[:3, 3]

            e_p = self.POS_GAIN * (self._T_des[:3, 3] - p_ee)
            e_r = self.ROT_GAIN * rot_error(T_cur[:3, :3], self._T_des[:3, :3])

            err_norm = np.sqrt(np.linalg.norm(e_p)**2 + np.linalg.norm(e_r)**2)
            if (np.linalg.norm(e_p) < self.TOL_POS and
                np.linalg.norm(e_r) < self.TOL_ROT):
                self._active = False
                break

            lam = self.LAMBDA_MIN + (self.LAMBDA_MAX - self.LAMBDA_MIN) * \
                  min(1.0, err_norm / self.LAMBDA_KNEE)

            M  = J @ J.T + (lam ** 2) * I6
            dq = J.T @ np.linalg.solve(M, np.r_[e_p, e_r])

            Jp  = J.T @ np.linalg.solve(M, J)
            dq += (In - Jp) @ (self.NULL_K * (self._chain.q_mid - self._q))

            mag = np.linalg.norm(dq)
            if mag > self.DQ_MAX:
                dq *= (self.DQ_MAX / mag)

            self._q = np.clip(self._q + dq, self._chain.q_min, self._chain.q_max)

        cmd              = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name         = self._names_in
        cmd.position     = self._q.tolist()
        self._pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(IKNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
