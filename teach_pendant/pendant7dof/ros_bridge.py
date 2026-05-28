"""ROS 2 bridge node used by the teach pendant GUI.

This is the single rclpy node the GUI owns. It mirrors the proven wiring from
the project's ``drawing_ui_node`` (jog / set / home / freeze + drawing) and
adds the teach-pendant extras: Cartesian jog via ``/ee_target``, named pose
presets, live status (joint positions + end-effector pose), and an E-stop that
freezes the arm and stops feeding any motion targets.
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, PoseStamped, Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [
    "joint_1", "joint_2", "joint_3", "joint_4",
    "joint_5", "joint_6", "joint_7",
]
HOME_JOINTS = [0.0] * 7
JOG_STEP_RAD = 0.1
JOG_DURATION_S = 0.5

# Soft joint limits kept just inside the URDF hard limits (matches
# drawing_ui_node). ``None`` = continuous joint.
LIMIT_MARGIN_RAD = 0.05
JOINT_LIMITS = [
    None,            # joint_1 — continuous
    (-1.6, 1.6),     # joint_2
    None,            # joint_3 — continuous
    (-1.6, 1.6),     # joint_4
    None,            # joint_5 — continuous
    (-0.48, 0.26),   # joint_6 — narrow range
    (-1.6, 1.6),     # joint_7
]

# Named joint presets shown as buttons in the GUI.
PRESETS: dict[str, list[float]] = {
    "Home": list(HOME_JOINTS),
    "Ready": [0.0, 0.4, 0.0, -0.6, 0.0, 0.0, 0.0],
}

# Cartesian jog step (metres) and the frame targets are published in.
CART_STEP_M = 0.01
BASE_FRAME = "base_link"


def clamp_joint(idx: int, value: float) -> float:
    lim = JOINT_LIMITS[idx]
    if lim is None:
        return value
    lo, hi = lim
    return max(lo + LIMIT_MARGIN_RAD, min(hi - LIMIT_MARGIN_RAD, value))


class PendantBridge(Node):
    def __init__(self) -> None:
        super().__init__("pendant7dof_bridge")

        # --- publishers ---------------------------------------------------
        self.traj_pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.target_pub = self.create_publisher(PoseStamped, "/ee_target", 10)
        self.strokes_pub = self.create_publisher(String, "drawing/strokes", 10)
        # Publishing an empty PoseArray clears any in-flight drawing path so
        # its IK output stops fighting manual commands.
        self.path_pub = self.create_publisher(PoseArray, "/cartesian_path", 10)

        # --- state --------------------------------------------------------
        self._last_q = list(HOME_JOINTS)
        self._ee_pose: PoseStamped | None = None
        self._estopped = False
        self._last_drawing: str | None = None

        # --- subscriptions ------------------------------------------------
        self.create_subscription(JointState, "/joint_states", self._cb_state, 20)
        self.create_subscription(PoseStamped, "/ee_pose", self._cb_ee, 20)

        self._pen_cb = None
        self.create_subscription(Point, "/pen_canvas_norm", self._cb_pen, 30)

        self.get_logger().info("pendant7dof bridge ready")

    # ── callbacks ─────────────────────────────────────────────────────────
    def _cb_state(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        for jn, p in zip(msg.name, msg.position):
            if jn in JOINT_NAMES:
                self._last_q[JOINT_NAMES.index(jn)] = float(p)

    def _cb_ee(self, msg: PoseStamped) -> None:
        self._ee_pose = msg

    def _cb_pen(self, msg: Point) -> None:
        if self._pen_cb is not None:
            self._pen_cb(float(msg.x), float(msg.y), float(msg.z))

    def attach_pen_callback(self, cb) -> None:
        self._pen_cb = cb

    # ── status accessors (read by the GUI) ────────────────────────────────
    def get_joint(self, idx: int) -> float:
        return float(self._last_q[idx])

    def get_joints(self) -> list[float]:
        return list(self._last_q)

    def get_ee_xyz(self) -> tuple[float, float, float] | None:
        if self._ee_pose is None:
            return None
        p = self._ee_pose.pose.position
        return (p.x, p.y, p.z)

    @property
    def estopped(self) -> bool:
        return self._estopped

    # ── motion helpers ────────────────────────────────────────────────────
    def _clear_path(self) -> None:
        empty = PoseArray()
        empty.header.stamp = self.get_clock().now().to_msg()
        empty.header.frame_id = BASE_FRAME
        self.path_pub.publish(empty)

    def _send_traj(self, positions: list[float], duration_s: float) -> None:
        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        sec = int(duration_s)
        pt.time_from_start = Duration(
            sec=sec, nanosec=int((duration_s - sec) * 1e9)
        )
        traj.points.append(pt)
        self.traj_pub.publish(traj)
        self._last_q = list(positions)

    def jog_joint(self, idx: int, delta: float) -> None:
        if self._estopped:
            self.get_logger().warn("E-stop active — jog ignored")
            return
        self._clear_path()
        q = list(self._last_q)
        q[idx] = clamp_joint(idx, q[idx] + delta)
        self._send_traj(q, JOG_DURATION_S)
        self.get_logger().info(
            f"JOG {JOINT_NAMES[idx]} {delta:+.2f} -> {q[idx]:+.3f} rad"
        )

    def set_joint(self, idx: int, target_rad: float, duration_s: float = 1.5) -> None:
        if self._estopped:
            self.get_logger().warn("E-stop active — set ignored")
            return
        self._clear_path()
        clamped = clamp_joint(idx, float(target_rad))
        if clamped != float(target_rad):
            self.get_logger().warn(
                f"{JOINT_NAMES[idx]}: {target_rad:+.3f} clamped to {clamped:+.3f}"
            )
        q = list(self._last_q)
        q[idx] = clamped
        self._send_traj(q, duration_s)

    def goto_preset(self, name: str, duration_s: float = 2.0) -> None:
        if self._estopped:
            self.get_logger().warn("E-stop active — preset ignored")
            return
        if name not in PRESETS:
            self.get_logger().warn(f"unknown preset '{name}'")
            return
        self._clear_path()
        self._send_traj(PRESETS[name], duration_s)
        self.get_logger().info(f"PRESET {name}")

    def send_home(self) -> None:
        self.goto_preset("Home")

    def cartesian_jog(self, axis: str, delta: float) -> None:
        """Nudge the end-effector target along a base-frame axis.

        Requires the IK node (consuming ``/ee_target``) and the FK node
        (publishing ``/ee_pose``) to be running so we have a current pose to
        offset from.
        """
        if self._estopped:
            self.get_logger().warn("E-stop active — cartesian jog ignored")
            return
        if self._ee_pose is None:
            self.get_logger().warn("no /ee_pose yet — is the FK node running?")
            return
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = BASE_FRAME
        target.pose = self._ee_pose.pose  # copy current orientation + position
        if axis == "x":
            target.pose.position.x += delta
        elif axis == "y":
            target.pose.position.y += delta
        elif axis == "z":
            target.pose.position.z += delta
        else:
            self.get_logger().warn(f"unknown axis '{axis}'")
            return
        self.target_pub.publish(target)
        self.get_logger().info(f"CART {axis}{delta:+.3f} m")

    # ── drawing ───────────────────────────────────────────────────────────
    def send_drawing(self, strokes_dict: dict) -> None:
        if self._estopped:
            self.get_logger().warn("E-stop active — drawing ignored")
            return
        msg = String()
        msg.data = json.dumps(strokes_dict)
        self.strokes_pub.publish(msg)
        self._last_drawing = msg.data
        n = len(strokes_dict.get("strokes", []))
        self.get_logger().info(f"published drawing: {n} strokes")

    def resend_last_drawing(self) -> None:
        if self._last_drawing is None:
            self.get_logger().warn("no previous drawing to resume")
            return
        msg = String()
        msg.data = self._last_drawing
        self.strokes_pub.publish(msg)

    # ── safety ────────────────────────────────────────────────────────────
    def freeze(self) -> None:
        """Preempt whatever the controller is doing and hold position."""
        self._clear_path()
        self._send_traj(list(self._last_q), 0.1)

    def estop(self) -> None:
        self._estopped = True
        self.freeze()
        self.get_logger().error("E-STOP engaged — arm frozen, commands blocked")

    def estop_reset(self) -> None:
        self._estopped = False
        self.get_logger().warn("E-stop cleared")
