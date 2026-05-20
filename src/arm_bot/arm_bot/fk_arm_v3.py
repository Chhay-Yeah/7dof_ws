#!/usr/bin/env python3
"""
fk_arm_v3.py  —  7-DOF FK node, URDF-driven.

FK is built directly from the URDF on /robot_description, so the kinematic
model matches what robot_state_publisher/RViz visualise exactly (no DH
idealisation, no per-joint axis-sign assumptions).

ROS interface:
  SUB  /joint_states      sensor_msgs/JointState     — current joint angles
  SUB  /robot_description std_msgs/String (latched)  — URDF
  PUB  /ee_pose           geometry_msgs/PoseStamped  — current EE pose
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from urdf_parser_py.urdf import URDF


# ── URDF-driven kinematic chain ──────────────────────────────────────────────

def _rpy_to_R(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def _origin_T(xyz, rpy) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _rpy_to_R(*rpy)
    T[:3,  3] = xyz
    return T


def _axis_angle_R(axis: np.ndarray, angle: float) -> np.ndarray:
    a = axis / np.linalg.norm(axis)
    x, y, z = a
    c, s = np.cos(angle), np.sin(angle)
    C = 1 - c
    return np.array([
        [c + x*x*C,     x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s,   c + y*y*C,   y*z*C - x*s],
        [z*x*C - y*s,   z*y*C + x*s, c + z*z*C ],
    ])


class UrdfChain:
    """Serial-chain FK built from a URDF."""

    def __init__(self, urdf_xml: str, base: str, tip: str):
        robot = URDF.from_xml_string(urdf_xml)

        parent_of = {j.child: (j, j.parent) for j in robot.joints}

        chain = []
        link = tip
        while link != base:
            if link not in parent_of:
                raise RuntimeError(f"link '{link}' has no parent (chain to '{base}' broken)")
            joint, parent = parent_of[link]
            chain.append(joint)
            link = parent
        chain.reverse()

        self.joints = []
        self.joint_names = []

        for j in chain:
            xyz = list(j.origin.xyz) if j.origin and j.origin.xyz else [0, 0, 0]
            rpy = list(j.origin.rpy) if j.origin and j.origin.rpy else [0, 0, 0]
            T_origin = _origin_T(xyz, rpy)
            axis = np.array(j.axis if j.axis is not None else [0, 0, 1], dtype=float)
            entry = {
                "name": j.name,
                "type": j.type,
                "T_origin": T_origin,
                "axis": axis,
            }
            self.joints.append(entry)
            if j.type in ("revolute", "continuous"):
                self.joint_names.append(j.name)

        self.n = len(self.joint_names)

    def fk(self, q: np.ndarray) -> np.ndarray:
        T = np.eye(4)
        qi = 0
        for j in self.joints:
            T = T @ j["T_origin"]
            if j["type"] in ("revolute", "continuous"):
                R = _axis_angle_R(j["axis"], q[qi])
                Rh = np.eye(4)
                Rh[:3, :3] = R
                T = T @ Rh
                qi += 1
        return T


# ── Math helpers ─────────────────────────────────────────────────────────────

def rot_to_quat(R: np.ndarray) -> np.ndarray:
    tr = np.trace(R)
    if tr > 0.0:
        S  = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            S  = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif i == 1:
            S  = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S  = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
    return np.array([qx, qy, qz, qw], dtype=float)


# ── ROS 2 node ───────────────────────────────────────────────────────────────

class FKNode(Node):
    def __init__(self):
        super().__init__("fk_7dof_v3")

        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("tip_link",  "ee")
        self._base = self.get_parameter("base_link").value
        self._tip  = self.get_parameter("tip_link").value

        self._chain: UrdfChain | None = None
        self._joint_index = None

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(String, "/robot_description", self._cb_urdf, latched)
        self.create_subscription(JointState, "/joint_states", self._cb_joints, 30)
        self._pub = self.create_publisher(PoseStamped, "/ee_pose", 30)

        self.get_logger().info(
            f"fk_arm_v3 ready — waiting for /robot_description (chain {self._base} → {self._tip})"
        )

    def _cb_urdf(self, msg: String):
        if self._chain is not None:
            return
        try:
            chain = UrdfChain(msg.data, self._base, self._tip)
        except Exception as e:
            self.get_logger().error(f"URDF parse failed: {e}")
            return
        self._chain = chain
        self.get_logger().info(
            f"URDF loaded: {chain.n} DoF — joints: {chain.joint_names}"
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

        q    = np.array([msg.position[i] for i in self._joint_index], dtype=float)
        T    = self._chain.fk(q)
        p    = T[:3, 3]
        quat = rot_to_quat(T[:3, :3])

        out = PoseStamped()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = self._base

        out.pose.position.x    = float(p[0])
        out.pose.position.y    = float(p[1])
        out.pose.position.z    = float(p[2])
        out.pose.orientation.x = float(quat[0])
        out.pose.orientation.y = float(quat[1])
        out.pose.orientation.z = float(quat[2])
        out.pose.orientation.w = float(quat[3])

        self._pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(FKNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
