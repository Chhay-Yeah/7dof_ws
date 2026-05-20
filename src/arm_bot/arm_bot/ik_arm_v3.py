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
    """Serial-chain FK + geometric Jacobian built from a URDF."""

    def __init__(self, urdf_xml: str, base: str, tip: str):
        robot = URDF.from_xml_string(urdf_xml)

        # parent map: child_link -> (joint, parent_link)
        parent_of = {j.child: (j, j.parent) for j in robot.joints}

        # walk tip → base
        chain = []
        link = tip
        while link != base:
            if link not in parent_of:
                raise RuntimeError(f"link '{link}' has no parent (chain to '{base}' broken)")
            joint, parent = parent_of[link]
            chain.append(joint)
            link = parent
        chain.reverse()  # base → tip order

        self.joints = []           # all joints in chain (fixed + revolute)
        self.q_joints = []         # indices into self.joints that are revolute/continuous
        self.joint_names = []      # names of revolute joints, in chain order

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
                self.q_joints.append(len(self.joints) - 1)
                self.joint_names.append(j.name)

        self.n = len(self.q_joints)

    def fk(self, q: np.ndarray):
        """Return list of T_world_at_joint_origin (before rotation) for every
        joint, plus T_ee."""
        T = np.eye(4)
        T_at_origin = []
        qi = 0
        for j in self.joints:
            T = T @ j["T_origin"]
            T_at_origin.append(T.copy())
            if j["type"] in ("revolute", "continuous"):
                R = _axis_angle_R(j["axis"], q[qi])
                Rh = np.eye(4)
                Rh[:3, :3] = R
                T = T @ Rh
                qi += 1
        return T_at_origin, T

    def jacobian(self, q: np.ndarray):
        T_at_origin, T_ee = self.fk(q)
        p_ee = T_ee[:3, 3]
        J = np.zeros((6, self.n))
        for col, joint_idx in enumerate(self.q_joints):
            T_o = T_at_origin[joint_idx]
            z = T_o[:3, :3] @ self.joints[joint_idx]["axis"]
            z = z / np.linalg.norm(z)
            o = T_o[:3, 3]
            J[:3, col] = np.cross(z, p_ee - o)
            J[3:, col] = z
        return J, T_ee


# ── Math helpers ─────────────────────────────────────────────────────────────

def rot_error(R_cur: np.ndarray, R_des: np.ndarray) -> np.ndarray:
    E = R_cur.T @ R_des
    return 0.5 * np.array([E[2, 1] - E[1, 2],
                           E[0, 2] - E[2, 0],
                           E[1, 0] - E[0, 1]])


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    x, y, z, w = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


# ── ROS 2 node ───────────────────────────────────────────────────────────────

class IKNode(Node):
    RATE_HZ      = 50.0
    LAMBDA_MAX   = 0.05      # damping when far from goal (numerical safety)
    LAMBDA_MIN   = 5e-4      # damping when close (lets residual shrink to ~µm)
    LAMBDA_KNEE  = 0.05      # error norm at which damping starts to shrink
    POS_GAIN     = 1.0        # proportional gain on position error
    ROT_GAIN     = 1.0        # proportional gain on rotation error
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
        self._q_mid = None
        self._q_min = None
        self._q_max = None
        self._joint_index = None   # index map: chain joint k → /joint_states index

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

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _cb_urdf(self, msg: String):
        if self._chain is not None:
            return
        try:
            chain = UrdfChain(msg.data, self._base, self._tip)
        except Exception as e:
            self.get_logger().error(f"URDF parse failed: {e}")
            return

        # Pull joint limits from URDF
        robot = URDF.from_xml_string(msg.data)
        limits = {j.name: j.limit for j in robot.joints
                  if j.type in ("revolute", "continuous") and j.limit is not None}

        q_min = np.full(chain.n, -np.pi)
        q_max = np.full(chain.n,  np.pi)
        for i, name in enumerate(chain.joint_names):
            lim = limits.get(name)
            if lim is not None and lim.lower is not None and lim.upper is not None:
                q_min[i] = lim.lower
                q_max[i] = lim.upper

        self._chain = chain
        self._q_min = q_min
        self._q_max = q_max
        self._q_mid = 0.5 * (q_min + q_max)
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

    # ── Control loop ───────────────────────────────────────────────────────

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

            # Adaptive damping: shrink λ as we approach the goal so the residual
            # can decay below mm-scale. λ scales linearly with err up to KNEE.
            lam = self.LAMBDA_MIN + (self.LAMBDA_MAX - self.LAMBDA_MIN) * \
                  min(1.0, err_norm / self.LAMBDA_KNEE)

            M  = J @ J.T + (lam ** 2) * I6
            dq = J.T @ np.linalg.solve(M, np.r_[e_p, e_r])

            # Null-space joint-centering (secondary task)
            Jp  = J.T @ np.linalg.solve(M, J)
            dq += (In - Jp) @ (self.NULL_K * (self._q_mid - self._q))

            # Per-step magnitude cap to prevent overshoot near singularities
            mag = np.linalg.norm(dq)
            if mag > self.DQ_MAX:
                dq *= (self.DQ_MAX / mag)

            self._q = np.clip(self._q + dq, self._q_min, self._q_max)

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
