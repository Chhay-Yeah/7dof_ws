#!/usr/bin/env python3
"""
drawing_batch_planner.py

Subscribes:
  /drawing/strokes    (std_msgs/String, JSON)
  /joint_states       (sensor_msgs/JointState) — for current-pose anchor + IK seed
  /robot_description  (std_msgs/String, latched) — URDF for FK/IK chain
Publishes:
  /arm_controller/joint_trajectory  (trajectory_msgs/JointTrajectory)

Per drawing message:
  - Fit a cubic spline through each stroke's pen-down (x, y) samples
    parameterized by cumulative chord length.
  - Resample at uniform spacing in workspace (mm).
  - Build a Cartesian waypoint list: approach → draw → lift → travel.
  - IK every waypoint with warm-seeding from the previous solution.
  - Time-parameterize at a constant Cartesian speed.
  - Publish ONE JointTrajectory containing all strokes for arm_controller
    to track. JointTrajectoryController interpolates between waypoints.
"""
import json
import threading
from typing import Optional

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseArray, Pose, Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from scipy.interpolate import CubicSpline
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from arm_bot.ik_lib import UrdfChain, solve_ik, rot_error


# ── Quaternion helpers (w, x, y, z convention used internally) ───────────────

def _slerp(q0, q1, t):
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1, dot = -q1, -dot
    if dot > 0.9995:
        r = q0 + t * (q1 - q0)
        return r / np.linalg.norm(r)
    theta_0 = np.arccos(dot)
    sin_t0  = np.sin(theta_0)
    theta   = theta_0 * t
    s0 = np.cos(theta) - dot * np.sin(theta) / sin_t0
    s1 = np.sin(theta) / sin_t0
    return s0 * q0 + s1 * q1


def _quat_wxyz_to_R(q):
    """Quaternion in (w, x, y, z) → 3×3 rotation matrix."""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def _R_to_quat_wxyz(R):
    """3×3 rotation matrix → quaternion (w, x, y, z). Shepperd's method."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z])


def _pose_T(x_m, y_m, z_m, q_wxyz):
    T = np.eye(4)
    T[:3, :3] = _quat_wxyz_to_R(q_wxyz)
    T[:3,  3] = [x_m, y_m, z_m]
    return T


class DrawingBatchPlanner(Node):
    def __init__(self):
        super().__init__('drawing_batch_planner')

        # ── Begin-draw pose ────────────────────────────────────────────────
        # Joint angles defining the "ready to draw" pose: robot leans forward
        # and down so the pen tip rests just above the paper. The pen-tip
        # position at this pose becomes the paper frame's origin; the EE
        # orientation at this pose becomes the constant drawing orientation.
        self.declare_parameter('begin_draw_joints',
                               [0.0, 0.9, 0.0, -1.2, 0.0, 0.01, 0.0])
        # The pen extends along the EE link's local +Z by this distance. The
        # EE's "ee" frame in the URDF is at joint 7, so we model the pen as a
        # 10 cm virtual extension since there's no gripper.
        self.declare_parameter('pen_offset_mm',          500.0)
        # Which EE-local axis the pen extends along. Default (0,0,1) means
        # "pen sticks out along EE +Z". For URDFs where the tool axis is
        # along +X (common with SolidWorks-imported arms), set this to
        # (1,0,0). The prober reports the three EE-local axes in base
        # frame so you can pick whichever one points down.
        self.declare_parameter('pen_axis_local',         [0.0, 0.0, 1.0])
        # Pre-drawing motion: joint-space move to begin_draw, then dwell.
        self.declare_parameter('move_to_begin_seconds',  4.0)
        self.declare_parameter('dwell_seconds',          3.0)

        # ── Workspace mapping (paper-relative) ─────────────────────────────
        # Canvas centred on the pen tip at begin_draw. These are the paper
        # plane extents, not absolute base-frame coords.
        self.declare_parameter('workspace_x_mm',         100.0)
        self.declare_parameter('workspace_y_mm',         100.0)
        # Pen-up clearance. POSITIVE lifts the pen UP, away from the paper;
        # 0 = no lift (pen stays at paper height). NEGATIVE would drive the
        # pen below the table — that target is unreachable in the drawing
        # posture and makes the IK flip the wrist to reach for it (looks like
        # the arm "glancing up" mid-stroke), so don't use negative values.
        self.declare_parameter('lift_mm',                0.0)
        # Shifts the paper plane up/down (base +Z) from the begin-draw pen-tip
        # height. 0 = draw at the begin pose's natural height; positive lifts
        # the whole plane, negative presses lower. The teach-pendant Drawing
        # settings drive this per-drawing via the message `config` block.
        self.declare_parameter('z_paper_offset_mm',      0.0)
        # Safety clamp: the reachable centred drawing square. Larger boxes push
        # joint_6 (tight limits) to its stops at the corners and IK degrades,
        # so GUI-supplied workspace sizes are clamped to this.
        self.declare_parameter('max_workspace_mm',       50.0)

        # ── Trajectory shaping ─────────────────────────────────────────────
        self.declare_parameter('sample_spacing_mm',      2.0)    # along stroke
        self.declare_parameter('draw_speed_mm_s',        15.0)
        # Between-stroke horizontal travel speed. Defaults to draw_speed
        # so multi-stroke drawings move at one steady speed; override in
        # the launch file if you want fast travel for big gaps.
        self.declare_parameter('travel_speed_mm_s',      15.0)
        self.declare_parameter('approach_seconds',       1.0)
        self.declare_parameter('approach_samples',       8)
        self.declare_parameter('initial_settle_seconds', 0.5)
        # Canvas → paper plane rotation, one of {0, 90, 180, 270}. Apply
        # around base +Z. 0 = canvas X to base X, canvas Y to base Y.
        # 90 / -90 swap them; 180 just reverses traversal direction (for
        # symmetric shapes like ellipses, 180 is invisible — try 90 or 270
        # instead). Combine with paper_mirror_x to flip handedness.
        self.declare_parameter('paper_rotation_deg',     90)
        # If True, additionally negate the X axis of the paper plane —
        # gives the 4 mirror orientations beyond the 4 rotations
        # (8 total). Use this if rotating alone never matches what you
        # see on the canvas (i.e. you need a reflection, not a rotation).
        self.declare_parameter('paper_mirror_x',         False)

        # ── Kinematics ─────────────────────────────────────────────────────
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('tip_link',  'ee')
        self.declare_parameter('frame_id',  'base_link')

        # ── Locked joints ──────────────────────────────────────────────────
        # Integer indices (0-based) of joints to FREEZE at their begin_draw
        # value during IK. Drawing then only uses the remaining "active"
        # joints. Use this to simplify which joints actually move.
        #
        # Joint index ↔ name mapping for this URDF:
        #   0 → joint_1   1 → joint_2   2 → joint_3   3 → joint_4
        #   4 → joint_5   5 → joint_6   6 → joint_7
        #
        # Example: lock joints 2,3,5,6 (only joints 1,4,7 active) =
        #          locked_joints: [1, 2, 4, 5]
        #
        # NOTE: with fewer than 6 active joints the IK is under-determined
        # — it gives a best-fit solution rather than exact tracking.
        # Drawing accuracy degrades to the residual size (visible in
        # `max_residual` in the trajectory log).
        #
        # ROS quirk: parameter list defaults can't be empty, so [-1] is
        # the sentinel for "no joints locked".
        self.declare_parameter('locked_joints', [-1])

        # ── IK shaping ─────────────────────────────────────────────────────
        # Null-space pull toward begin_draw. Higher = stronger rubber band
        # keeping the IK solution near q_begin across the whole drawing.
        # 2.0 matches workspace_prober.py; 10+ helps suppress branch jumps
        # at the edge of the reachable workspace.
        self.declare_parameter('null_k', 2.0)
        # Per-joint movement penalty for weighted DLS. Higher weight = that
        # joint moves less. Use to push the solver away from joints with
        # tight limits (joint_6 here is [-0.489, +0.262]).
        # Examples:
        #   [1, 1, 1, 1, 1, 1, 1]  → uniform; matches workspace_prober.py
        #   [1, 1, 1, 1, 1, 5, 1]  → penalize joint_6 5×
        self.declare_parameter('joint_weights', [1.0] * 7)

        # ── Debug logging ──────────────────────────────────────────────────
        # When True, dump every trajectory point with its joint values and
        # the per-joint delta from the previous point. Verbose (one line
        # per waypoint, ~20–100 lines per drawing), so off by default.
        self.declare_parameter('log_joint_deltas', False)

        gp = lambda n: self.get_parameter(n).value
        self.begin_draw_joints = np.array(gp('begin_draw_joints'), dtype=float)
        self.pen_offset_m      = float(gp('pen_offset_mm')) / 1000.0
        self.pen_axis_local    = np.array(gp('pen_axis_local'), dtype=float)
        self.pen_axis_local   /= np.linalg.norm(self.pen_axis_local)
        self.t_move_to_begin   = float(gp('move_to_begin_seconds'))
        self.t_dwell           = float(gp('dwell_seconds'))
        self.wx, self.wy       = gp('workspace_x_mm'), gp('workspace_y_mm')
        self.lift_mm           = float(gp('lift_mm'))
        self.z_paper_offset_mm = float(gp('z_paper_offset_mm'))
        self.max_workspace_mm  = float(gp('max_workspace_mm'))
        self.ds_mm             = float(gp('sample_spacing_mm'))
        self.v_draw            = float(gp('draw_speed_mm_s'))
        self.v_travel          = float(gp('travel_speed_mm_s'))
        self.t_approach        = float(gp('approach_seconds'))
        self.n_approach        = int(gp('approach_samples'))
        self.t_settle          = float(gp('initial_settle_seconds'))
        self.base_link         = gp('base_link')
        self.tip_link          = gp('tip_link')
        self.frame_id          = gp('frame_id')
        self.log_joint_deltas  = bool(gp('log_joint_deltas'))
        self.locked_joints     = [int(i) for i in gp('locked_joints')
                                  if int(i) >= 0]
        self.null_k            = float(gp('null_k'))
        self.joint_weights     = [float(w) for w in gp('joint_weights')]
        self.paper_rotation_deg = int(gp('paper_rotation_deg'))
        self.paper_mirror_x    = bool(gp('paper_mirror_x'))

        # Paper-frame state (filled in from FK at begin_draw on every call)
        self.ox = self.oy = self.z_paper = self.z_lift = 0.0
        self.q_draw     = np.array([0.0, 1.0, 0.0, 0.0])
        # No orientation change during approach — constant lean-forward pose.
        self.q_approach = self.q_draw.copy()

        # ── Runtime state ──────────────────────────────────────────────────
        self._chain: Optional[UrdfChain] = None
        self._q_current: Optional[np.ndarray] = None
        self._joint_index = None
        self._lock = threading.Lock()

        # Paper-frame anchor — computed once the URDF arrives so the live
        # pen-position broadcast can convert base-frame FK back to canvas
        # coords without waiting for the first /drawing/strokes message.
        # We store the PEN TIP at begin_draw (not the EE origin) because
        # the planner positions the EE for each waypoint while the pen
        # tip is offset by pen_offset_m * (R_begin @ pen_axis_local) — a
        # fixed vector in base frame as long as the wrist orientation is
        # held constant. Anchoring to the pen tip means canvas centre
        # (norm = 0.5, 0.5) corresponds to the actual pen tip rest pose.
        self._pen_anchor_base: Optional[np.ndarray] = None  # (3,) base m
        self._paper_R_persistent: Optional[np.ndarray] = None  # (3,3)

        # ── ROS interfaces ─────────────────────────────────────────────────
        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(String, '/robot_description', self._cb_urdf, latched)
        self.create_subscription(JointState, '/joint_states', self._cb_joints, 30)
        self.create_subscription(String, '/drawing/strokes', self._cb_drawing, 10)

        self._traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        # Also publish the Cartesian path for visualization / debugging
        self._path_pub = self.create_publisher(PoseArray, '/cartesian_path', 10)
        # Live pen-tip position in normalized canvas coords [0..1]^2.
        # x = right, y = up. The UI subscribes to draw a tracking dot.
        # z carries the pen height in mm above the paper plane (sign:
        # positive = above paper, negative = below / pressed into paper).
        self._pen_pub = self.create_publisher(Point, '/pen_canvas_norm', 10)

        self.get_logger().info(
            f'drawing_batch_planner ready — chain {self.base_link} → {self.tip_link}'
        )

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _cb_urdf(self, msg: String):
        if self._chain is not None:
            return
        try:
            chain = UrdfChain(msg.data, self.base_link, self.tip_link)
        except Exception as e:
            self.get_logger().error(f'URDF parse failed: {e}')
            return
        self._chain = chain
        self.get_logger().info(
            f'URDF loaded: {chain.n} DoF — joints: {chain.joint_names}'
        )

        # Precompute paper-frame anchor (pen tip at begin_draw) and paper_R
        # so the live pen-position broadcast can run before any drawing
        # arrives. Same math as in _cb_drawing — kept in sync.
        if self.begin_draw_joints.shape[0] == chain.n:
            q_b = np.minimum(np.maximum(self.begin_draw_joints.copy(),
                                         chain.q_min), chain.q_max)
            _, T_b = chain.fk(q_b)
            R_b = T_b[:3, :3]
            pen_dir_b = R_b @ self.pen_axis_local
            self._pen_anchor_base = T_b[:3, 3] + self.pen_offset_m * pen_dir_b
            theta = np.radians(self.paper_rotation_deg)
            c, s = float(np.cos(theta)), float(np.sin(theta))
            sx = -1.0 if self.paper_mirror_x else 1.0
            self._paper_R_persistent = np.array([
                [c * sx, -s,  0.0],
                [s * sx,  c,  0.0],
                [0.0,     0.0, 1.0],
            ])

    def _cb_joints(self, msg: JointState):
        if self._chain is None:
            return
        if self._joint_index is None:
            try:
                self._joint_index = [msg.name.index(n)
                                     for n in self._chain.joint_names]
            except ValueError as e:
                self.get_logger().warn(
                    f'/joint_states missing one of {self._chain.joint_names}: {e}'
                )
                return
        q = np.array([msg.position[i] for i in self._joint_index], dtype=float)
        with self._lock:
            self._q_current = q

        # Broadcast pen-tip position in normalized canvas coords so the UI
        # can render a live tracking dot. Cheap to compute (FK + 3×3 mul).
        self._publish_pen_pos(q)

    def _publish_pen_pos(self, q: np.ndarray) -> None:
        """Compute pen-tip position from FK and publish normalized canvas
        coords on /pen_canvas_norm. x, y in [0..1] (left→right, bottom→top);
        z = mm above paper (positive = lifted, negative = pressed in)."""
        if self._pen_anchor_base is None or self._paper_R_persistent is None:
            return
        _, T_ee = self._chain.fk(q)
        pen_tip_base = T_ee[:3, 3] + self.pen_offset_m * (T_ee[:3, :3] @ self.pen_axis_local)
        # Offset from the pen-tip-at-begin-draw rest position. paper_R is
        # orthogonal so its inverse is its transpose. Anchoring to the pen
        # tip (not the EE origin) means a canvas point that the planner
        # treats as origin actually lands at norm = (0.5, 0.5).
        offset_base_m = pen_tip_base - self._pen_anchor_base
        paper_xyz_m = self._paper_R_persistent.T @ offset_base_m
        paper_xy_mm = paper_xyz_m[:2] * 1000.0
        # Normalize using workspace extents: 0 = left/bottom edge, 1 = right/top.
        # paper_xy_mm spans approximately [-wx/2..+wx/2] × [-wy/2..+wy/2].
        norm_x = float(paper_xy_mm[0] / self.wx + 0.5)
        norm_y = float(paper_xy_mm[1] / self.wy + 0.5)
        msg = Point()
        msg.x = norm_x
        msg.y = norm_y
        msg.z = float(paper_xyz_m[2] * 1000.0)
        self._pen_pub.publish(msg)

    def _cb_drawing(self, msg: String):
        if self._chain is None:
            self.get_logger().warn('No URDF yet — cannot plan')
            return
        with self._lock:
            q_start = None if self._q_current is None else self._q_current.copy()
        if q_start is None:
            self.get_logger().warn('No /joint_states yet — cannot plan')
            return

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Bad JSON: {e}')
            return

        # Per-drawing config from the teach pendant. The GUI is the single
        # source of truth so the canvas and the robot workspace stay in sync:
        # the same workspace_x/y the GUI sized the canvas to is what we map
        # into here, so a square drawn on screen is square on the table.
        cfg = data.get('config', {})
        if 'workspace_x_mm' in cfg:
            self.wx = min(float(cfg['workspace_x_mm']), self.max_workspace_mm)
        if 'workspace_y_mm' in cfg:
            self.wy = min(float(cfg['workspace_y_mm']), self.max_workspace_mm)
        if 'lift_mm' in cfg:
            # Negative lift/offset drives the pen below the table — unreachable
            # in the drawing posture and makes the IK flip the wrist mid-stroke.
            self.lift_mm = max(0.0, float(cfg['lift_mm']))
        if 'z_paper_offset_mm' in cfg:
            self.z_paper_offset_mm = max(0.0, float(cfg['z_paper_offset_mm']))

        strokes = data.get('strokes', [])
        if not strokes or all(not s.get('points') for s in strokes):
            self.get_logger().warn('Empty drawing — nothing to plan')
            return

        # Anchor the paper frame on the pen tip at begin_draw. EE orientation
        # at begin_draw becomes the constant drawing orientation.
        if self.begin_draw_joints.shape[0] != self._chain.n:
            self.get_logger().error(
                f'begin_draw_joints has {self.begin_draw_joints.shape[0]} entries; '
                f'URDF chain has {self._chain.n} DoF'
            )
            return

        q_begin = self.begin_draw_joints.copy()
        # Clamp to URDF limits so a misconfigured param can't drive the
        # controller out of bounds.
        q_begin = np.minimum(np.maximum(q_begin, self._chain.q_min),
                             self._chain.q_max)

        _, T_begin = self._chain.fk(q_begin)
        R_begin = T_begin[:3, :3]
        # Pen extends from EE along `pen_axis_local` in the EE local frame.
        pen_dir_base     = R_begin @ self.pen_axis_local
        pen_tip_at_begin = T_begin[:3, 3] + self.pen_offset_m * pen_dir_base

        # Paper plane is HORIZONTAL in base frame — drawing happens at
        # constant base z = T_begin.z, lift moves the EE up in base +Z.
        # The pen tip drags along the table at whatever tilt the wrist
        # imposes; this is what physically happens with a tilted pen on a
        # flat surface.
        #
        # Rotation around base +Z chosen by the launch param. 4 options:
        #   0   → paper +X = base +X, paper +Y = base +Y
        #   90  → paper +X = base +Y, paper +Y = base -X
        #   180 → paper +X = base -X, paper +Y = base -Y
        #   270 → paper +X = base -Y, paper +Y = base +X
        # With paper_mirror_x=True, paper X is additionally reflected
        # (gives the 4 mirrored orientations — handedness flip).
        theta = np.radians(self.paper_rotation_deg)
        c, s = float(np.cos(theta)), float(np.sin(theta))
        sx = -1.0 if self.paper_mirror_x else 1.0
        paper_R = np.array([
            [c * sx, -s,  0.0],
            [s * sx,  c,  0.0],
            [0.0,     0.0, 1.0],
        ])

        # Paper frame is anchored at T_begin.t with axes = EE local axes at
        # begin_draw (paper_R = R_begin). So paper +Z = pen direction (into
        # the paper) and lift = -paper_Z = away from pen direction. The IK
        # loop converts paper-frame waypoints (px, py, pz) in meters to
        # base-frame EE targets via:
        #   ee_target_base = T_begin.t + R_begin @ (px, py, pz)
        # Canvas centre (px=py=pz=0) gives ee_target_base = T_begin.t,
        # solved trivially by q_begin. Lift uses POSITIVE z because paper_R
        # = identity → paper +Z = base +Z = away from horizontal paper.
        self.ox      = -self.wx / 2.0
        self.oy      = -self.wy / 2.0
        self.z_paper = self.z_paper_offset_mm
        self.z_lift  = self.z_paper_offset_mm + self.lift_mm
        # Orientation stays = R_begin throughout. q_draw/q_approach are
        # kept identical so the SLERP in the approach/lift loops is a
        # no-op (orientation already correct from the joint-space move
        # to begin_draw). The pen tilt is set entirely by
        # begin_draw_joints[6] — change that to control how perpendicular
        # the pen sits during drawing.
        self.q_draw     = _R_to_quat_wxyz(R_begin)
        self.q_approach = self.q_draw.copy()

        cart_wps = self._build_cartesian_waypoints(data)
        if not cart_wps:
            self.get_logger().warn('Planner produced no Cartesian waypoints')
            return

        # Publish the Cartesian path for RViz (optional, no executor uses it)
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = self.frame_id
        for (x, y, z, q_wxyz, _dt, _kind) in cart_wps:
            p = Pose()
            p.position.x, p.position.y, p.position.z = x, y, z
            p.orientation.w = float(q_wxyz[0])
            p.orientation.x = float(q_wxyz[1])
            p.orientation.y = float(q_wxyz[2])
            p.orientation.z = float(q_wxyz[3])
            pa.poses.append(p)
        self._path_pub.publish(pa)

        # IK every waypoint, warm-seeded from the previous solution.
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(self._chain.joint_names)

        # Phase 1: current state → begin_draw (joint-space interpolation).
        # The controller cubic-blends between waypoints, so two endpoints
        # are enough for a smooth move.
        point_kinds: list[str] = []
        pt_start = JointTrajectoryPoint()
        pt_start.positions = q_start.tolist()
        pt_start.time_from_start = Duration(sec=0, nanosec=0)
        traj.points.append(pt_start)
        point_kinds.append('start')

        pt_begin = JointTrajectoryPoint()
        pt_begin.positions = q_begin.tolist()
        pt_begin.time_from_start = self._seconds_to_duration(self.t_move_to_begin)
        traj.points.append(pt_begin)
        point_kinds.append('begin_draw')

        # Phase 2: hold at begin_draw for dwell_seconds.
        t_drawing_start = self.t_move_to_begin + self.t_dwell
        pt_hold = JointTrajectoryPoint()
        pt_hold.positions = q_begin.tolist()
        pt_hold.time_from_start = self._seconds_to_duration(t_drawing_start)
        traj.points.append(pt_hold)
        point_kinds.append('hold')

        # Log paper anchor + pen direction so the user can sanity-check the
        # paper plane orientation. Pen direction = R_begin @ [0,0,1] in
        # base frame; the paper plane is perpendicular to this vector.
        pen_dir = R_begin @ self.pen_axis_local
        self.get_logger().info(
            f'begin_draw: EE at ({T_begin[0,3]:+.3f}, {T_begin[1,3]:+.3f}, '
            f'{T_begin[2,3]:+.3f}) m; pen tip at '
            f'({pen_tip_at_begin[0]:+.3f}, {pen_tip_at_begin[1]:+.3f}, '
            f'{pen_tip_at_begin[2]:+.3f}) m; '
            f'pen dir = ({pen_dir[0]:+.2f}, {pen_dir[1]:+.2f}, {pen_dir[2]:+.2f}) '
            f'(paper ⊥ this); paper box X=±{self.wx/2:.0f} mm, '
            f'Y=±{self.wy/2:.0f} mm, lift={-self.z_lift:.0f} mm away from paper'
        )

        # Batch IK params — match workspace_prober.py exactly so the
        # reachability map it generates is meaningful here. Non-uniform
        # joint_weights cause IK to push joint_6 to its tight limit
        # [-0.489, +0.262] at the workspace edge, branch-jumping the
        # posture (large Δq in mid-stroke).
        ik_params = dict(
            dq_max=0.05,
            max_iters=1200,
            null_k=self.null_k,
            q_null_target=q_begin,
            joint_weights=self.joint_weights,
            # Drawing-realistic tolerances. Default 1e-5 m / 1e-4 rad is
            # way too tight for DLS to converge in 600 iterations — leads
            # to spurious "IK failed" for reachable targets.
            tol_pos=1e-3,
            tol_rot=1e-2,
            locked_joints=self.locked_joints,
        )
        if self.locked_joints:
            active = [i for i in range(self._chain.n)
                      if i not in self.locked_joints]
            self.get_logger().info(
                f'Locked joints: {self.locked_joints} '
                f'(frozen at begin_draw value); active joints: {active}. '
                f'{"Under-determined IK — drawing may drift." if len(active) < 6 else ""}'
            )

        # Warm-start IK from begin_draw — drawing motion is local to it,
        # so seeding from there gives the cleanest null-space behavior.
        q_seed = q_begin
        cum_t = t_drawing_start + self.t_settle
        n_unconverged = 0
        max_resid = 0.0
        first_resid = None

        for idx, (x, y, z, q_wxyz, dt, _kind) in enumerate(cart_wps):
            # Treat (x, y, z) as paper-frame meters and transform to base
            # using paper_R (which has paper +Z aligned with the pen
            # direction in base). Orientation stays = R_begin throughout
            # drawing — the wrist tilt is set by begin_draw_joints[6] and
            # the IK keeps it there via the heavy joint_7 weight.
            ee_pos_base = T_begin[:3, 3] + paper_R @ np.array([x, y, z])
            T_des = np.eye(4)
            T_des[:3, :3] = R_begin
            T_des[:3,  3] = ee_pos_base
            # ALWAYS seed from q_begin and pull toward q_begin in null space.
            # The prober verified every cell in the 60 mm centred square
            # converges from this seed, so each waypoint lands in the same
            # branch. Warm-seeding from q_seed = previous solution let the
            # IK wander into a far-from-q_begin branch once mid-stroke and
            # then "stick" there because q_null_target tracked the bad
            # seed. Re-seeding kills that drift.
            ik_params_step = {**ik_params, 'q_null_target': q_begin}
            q_sol, resid, conv = solve_ik(
                self._chain, T_des, q_begin,
                use_null_space=True, params=ik_params_step,
            )
            point_kinds.append(_kind)

            if idx == 0:
                first_resid = resid
                # Detailed first-call diagnostic so unreachable targets are
                # immediately obvious from the log.
                _, T_final = self._chain.fk(q_sol)
                e_p = T_des[:3, 3] - T_final[:3, 3]
                e_r = rot_error(T_final[:3, :3], T_des[:3, :3])
                self.get_logger().info(
                    f'IK[0]: pos_err={np.linalg.norm(e_p)*1000:.1f} mm, '
                    f'rot_err={np.linalg.norm(e_r):.3f} rad; '
                    f'EE ended at ({T_final[0,3]:+.3f}, {T_final[1,3]:+.3f}, '
                    f'{T_final[2,3]:+.3f}) m; '
                    f'q_sol=[{", ".join(f"{v:+.2f}" for v in q_sol)}]'
                )
                # Flag joints sitting at their limits (likely culprit)
                at_lo = np.where(q_sol - self._chain.q_min < 1e-3)[0]
                at_hi = np.where(self._chain.q_max - q_sol < 1e-3)[0]
                if len(at_lo) or len(at_hi):
                    names = list(self._chain.joint_names)
                    pinned = [f'{names[i]}@min' for i in at_lo] + \
                             [f'{names[i]}@max' for i in at_hi]
                    self.get_logger().warn(
                        f'Joints pinned to limits after IK: {pinned}'
                    )

            if not conv:
                n_unconverged += 1
            max_resid = max(max_resid, resid)

            cum_t += dt
            pt = JointTrajectoryPoint()
            pt.positions = q_sol.tolist()
            pt.time_from_start = self._seconds_to_duration(cum_t)
            traj.points.append(pt)

            q_seed = q_sol

        # Refuse to publish a useless trajectory — if IK can't reach the first
        # waypoint, the controller would just sit at q_start and the user
        # would see "robot doesn't move" with no clue why.
        if first_resid is not None and first_resid > 1e-2:
            self.get_logger().error(
                f'IK failed on first waypoint (residual={first_resid:.2e}). '
                f'Target likely unreachable from begin_draw. Try shrinking '
                f'workspace_x/y_mm, lowering lift_mm, or changing '
                f'begin_draw_joints so the pen tip sits closer to the centre '
                f'of the reachable workspace. NOT publishing trajectory.'
            )
            return

        if self.log_joint_deltas:
            self._dump_trajectory(traj, point_kinds)

        self._traj_pub.publish(traj)
        self.get_logger().info(
            f'→ JointTrajectory: {len(traj.points)} points, '
            f'duration={cum_t:.1f}s, first_residual={first_resid:.2e}, '
            f'max_residual={max_resid:.2e}, unconverged={n_unconverged}'
        )

    # ── Planning helpers ───────────────────────────────────────────────────

    @staticmethod
    def _seconds_to_duration(t_sec: float) -> Duration:
        sec = int(t_sec)
        nsec = int(round((t_sec - sec) * 1e9))
        return Duration(sec=sec, nanosec=nsec)

    def _dump_trajectory(self, traj: JointTrajectory, kinds: list) -> None:
        """Log every trajectory point with q values + per-joint Δq vs prev."""
        n = len(traj.points)
        if n == 0:
            return
        self.get_logger().info(
            f'--- trajectory dump ({n} points, {len(traj.joint_names)} joints) ---'
        )
        prev_q = None
        for i, (pt, kind) in enumerate(zip(traj.points, kinds)):
            q = np.asarray(pt.positions, dtype=float)
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            q_str = '[' + ', '.join(f'{v:+.3f}' for v in q) + ']'
            if prev_q is None:
                self.get_logger().info(
                    f'  [{i:03d}] t={t:6.2f}s {kind:>10}  q={q_str}'
                )
            else:
                dq = q - prev_q
                dq_str = '[' + ', '.join(f'{v:+.3f}' for v in dq) + ']'
                self.get_logger().info(
                    f'  [{i:03d}] t={t:6.2f}s {kind:>10}  '
                    f'q={q_str}  Δq={dq_str}  max|Δq|={np.max(np.abs(dq)):.3f}'
                )
            prev_q = q
        self.get_logger().info('--- end trajectory dump ---')

    def _build_cartesian_waypoints(self, data):
        """Returns a list of (x_m, y_m, z_m, q_wxyz, dt_s, kind) tuples.

        kind is one of "approach", "draw", "lift", "travel" — purely for logs.
        """
        cw = data['canvas']['width']
        ch = data['canvas']['height']
        sx = self.wx / cw
        sy = self.wy / ch

        wps = []
        prev_lift_xy = None  # (x_mm, y_mm) above end of last stroke

        for s_idx, stroke in enumerate(data['strokes']):
            pts = stroke.get('points', [])
            if not pts:
                continue

            # Canvas → workspace mm (flip Y so the robot Y axis points "up")
            xs = np.array([self.ox + p['x'] * sx for p in pts], dtype=float)
            ys = np.array([self.oy + (ch - p['y']) * sy for p in pts], dtype=float)

            # Dedup co-located points so the spline parameterization is monotonic
            keep = [0]
            for i in range(1, len(xs)):
                if (xs[i] - xs[keep[-1]])**2 + (ys[i] - ys[keep[-1]])**2 > 1e-8:
                    keep.append(i)
            xs, ys = xs[keep], ys[keep]
            if len(xs) < 2:
                xs_s, ys_s = xs.copy(), ys.copy()
            else:
                # Cumulative chord length parameter
                dx = np.diff(xs)
                dy = np.diff(ys)
                seg = np.hypot(dx, dy)
                u = np.concatenate(([0.0], np.cumsum(seg)))
                L = float(u[-1])
                if L < 1e-6:
                    xs_s, ys_s = xs.copy(), ys.copy()
                else:
                    # Resample at uniform chord spacing (close to arc length for
                    # smooth pen strokes).
                    n_samples = max(2, int(round(L / self.ds_mm)) + 1)
                    u_new = np.linspace(0.0, L, n_samples)
                    if len(xs) >= 4:
                        cs_x = CubicSpline(u, xs, bc_type='natural')
                        cs_y = CubicSpline(u, ys, bc_type='natural')
                        xs_s = cs_x(u_new)
                        ys_s = cs_y(u_new)
                    else:
                        # Too few points for cubic — linear is fine
                        xs_s = np.interp(u_new, u, xs)
                        ys_s = np.interp(u_new, u, ys)

            x0_mm, y0_mm = float(xs_s[0]),  float(ys_s[0])
            xN_mm, yN_mm = float(xs_s[-1]), float(ys_s[-1])

            # 1. Travel from last lift (or anchor) to over this stroke's start
            if prev_lift_xy is not None:
                px, py = prev_lift_xy
                dx, dy = (x0_mm - px), (y0_mm - py)
                dist = float(np.hypot(dx, dy))
                if dist > 1e-3:
                    n_travel = max(2, int(round(dist / self.ds_mm)) + 1)
                    ts = np.linspace(0.0, 1.0, n_travel)[1:]  # skip start (= prev_lift)
                    dt_each = (dist / self.v_travel) / max(1, len(ts))
                    for t in ts:
                        wps.append((
                            (px + dx * t) / 1000.0,
                            (py + dy * t) / 1000.0,
                            self.z_lift / 1000.0,
                            self.q_approach,
                            dt_each,
                            'travel',
                        ))

            # 2. Approach: descend from z_lift → z_paper while SLERPing
            #    orientation q_approach → q_draw, hovering over (x0, y0).
            n_app = max(2, self.n_approach)
            dt_app = self.t_approach / n_app
            # The first approach sample is the lift-over-start (we may already
            # be here from a previous lift, but include it for the first stroke).
            for i in range(1, n_app + 1):
                t = i / float(n_app)
                z = self.z_lift + (self.z_paper - self.z_lift) * t
                q = _slerp(self.q_approach, self.q_draw, t)
                wps.append((
                    x0_mm / 1000.0,
                    y0_mm / 1000.0,
                    z     / 1000.0,
                    q,
                    dt_app,
                    'approach',
                ))

            # 3. Draw the stroke at z_paper with constant orientation
            #    First point is the touch-down (already at z_paper, q_draw).
            #    Time per sample = chord_dist / v_draw.
            for i in range(1, len(xs_s)):
                dx = xs_s[i] - xs_s[i-1]
                dy = ys_s[i] - ys_s[i-1]
                d  = float(np.hypot(dx, dy))
                dt = d / self.v_draw if self.v_draw > 0 else 0.0
                wps.append((
                    float(xs_s[i]) / 1000.0,
                    float(ys_s[i]) / 1000.0,
                    self.z_paper   / 1000.0,
                    self.q_draw,
                    max(dt, 1e-3),
                    'draw',
                ))

            # 4. Lift: ascend z_paper → z_lift, SLERP q_draw → q_approach.
            n_lift = max(2, self.n_approach)
            dt_lift = self.t_approach / n_lift
            for i in range(1, n_lift + 1):
                t = i / float(n_lift)
                z = self.z_paper + (self.z_lift - self.z_paper) * t
                q = _slerp(self.q_draw, self.q_approach, t)
                wps.append((
                    xN_mm / 1000.0,
                    yN_mm / 1000.0,
                    z     / 1000.0,
                    q,
                    dt_lift,
                    'lift',
                ))

            prev_lift_xy = (xN_mm, yN_mm)

        return wps


def main(args=None):
    rclpy.init(args=args)
    node = DrawingBatchPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
