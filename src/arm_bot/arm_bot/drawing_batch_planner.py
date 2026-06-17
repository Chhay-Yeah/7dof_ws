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
import time
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

from arm_bot.ik_lib import UrdfChain, solve_ik, solve_ik_tip, rot_error


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
        # The pen length (mm) the begin pose was tuned for — defines the table
        # height. When the live pen_offset_mm differs (a longer/shorter pen),
        # the begin pose's EE is raised/lowered by the difference so the pen TIP
        # stays on the same table (otherwise the pen length cancels out and has
        # no effect on the motion). See the z-adjust in _cb_drawing.
        self.declare_parameter('pen_offset_ref_mm',      100.0)
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
        # Pen-up clearance, expressed as the tip→paper distance during travel.
        # SIGN: negative = pen UP (away from the paper); the magnitude is the
        # clearance in mm. So -25 means the pen rides 25 mm above the paper
        # between strokes. The clearance is floored at 10 mm (see z_lift below)
        # so the pen never reaches the table; there is no upper cap.
        self.declare_parameter('lift_mm',                -25.0)
        # Safety clamp: the reachable centred drawing square. Larger boxes push
        # joint_6 (tight limits) to its stops at the corners and IK degrades,
        # so GUI-supplied workspace sizes are clamped to this.
        self.declare_parameter('max_workspace_mm',       50.0)
        # Tilt-capable drawing. When True the IK controls the PEN TIP with a
        # task-priority solver: tip position is primary (so IK doesn't fail on
        # reachable points) and pen-perpendicular is secondary (held where the
        # arm has spare DOF, near the canvas centre; the pen tilts only as much
        # as the edges force it to). This expands the drawable area roughly
        # from a ~40 mm square to a ~260x110 mm rectangle. False (default)
        # keeps the original perpendicular-everywhere EE-origin solve.
        self.declare_parameter('draw_tilt',              False)
        # Re-centre the canvas on the reachable region: a paper-frame offset
        # (mm) added to every waypoint so the canvas centre maps into the
        # middle of the drawable area instead of the begin-pose pen tip (which
        # sits near the region's edge). 0,0 = canvas centred on the begin tip.
        self.declare_parameter('canvas_anchor_x_mm',     0.0)
        self.declare_parameter('canvas_anchor_y_mm',     0.0)
        # Diagnostic: if >0, log a warning when any waypoint's pen tilt exceeds
        # this (deg). 0 = uncapped (just report the max). Tilt is not clamped;
        # the canvas is sized so the worst-case tilt stays acceptable.
        self.declare_parameter('tilt_max_deg',           0.0)

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
        # Pen-DOWN hold at each stroke's start and end. After the pen descends
        # to the paper at the stroke start it holds here this many seconds before
        # dragging; after the stroke finishes it holds again before lifting.
        # 0 disables (no hold). Implemented as zero-motion waypoints, so the arm
        # fully stops (zero velocity) during the hold.
        self.declare_parameter('stroke_dwell_seconds',   2.0)
        # Sharp-corner handling. A single cubic spline through a whole stroke
        # rounds genuine corners (e.g. a rectangle's 90° turns) into smooth
        # arcs. Vertices whose turn angle (deviation from straight) exceeds this
        # are treated as corners: the spline is SPLIT there so each corner stays
        # an exact vertex with near-straight approaches. 0 disables (smooth the
        # whole stroke — the old behaviour).
        self.declare_parameter('corner_angle_deg',       40.0)
        # Pen-down hold (s) at each detected corner. With velocity feedforward
        # off the controller blends through the vertex (rounding it); this
        # zero-motion hold makes the arm fully stop and settle so the corner is
        # crisp. Long enough (~1 s) that the controller decelerates to rest at
        # the exact vertex before turning. 0 = geometry split only (softer).
        self.declare_parameter('corner_dwell_seconds',   1.0)
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
        # Table-tilt compensation. The drawing plane is otherwise perfectly
        # horizontal in the robot base frame (paper_R z-axis = base +Z). If the
        # real table is not parallel to the base XY plane, the pen digs in on the
        # high side and lifts on the low side. These tilt the drawing plane to
        # match the table so the pen tip keeps constant contact across the canvas:
        #   table_tilt_x_deg = rotation about base +X (compensates a base-Y slope)
        #   table_tilt_y_deg = rotation about base +Y (compensates a base-X slope)
        # 0/0 = horizontal (today's behaviour). Calibrate by touching the table at
        # a couple of points (see drawing_batch_planner docstring / the chat) and
        # flip the sign if it makes the dip worse.
        self.declare_parameter('table_tilt_x_deg',       0.0)
        self.declare_parameter('table_tilt_y_deg',       0.0)

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
        # Populate per-point joint velocities (finite-difference of positions
        # over the timestamps). This was a feedforward term for the OLD
        # velocity-command controller. The arm now runs a POSITION command
        # interface, where the controller only uses these velocities to shape
        # its cubic-spline interpolation between waypoints — and at an IK
        # config flip (a large Δq between adjacent points) the finite-diff
        # velocity is huge, so the spline OVERSHOOTS far past the waypoint and
        # the pen swings off the path. Default OFF: positions-only points let
        # the controller compute its own bounded spline velocities (smooth, no
        # overshoot). Only turn on for a velocity-command controller.
        self.declare_parameter('velocity_ff', False)
        # Re-time any trajectory segment that would demand more than this
        # joint speed (rad/s), by stretching its time_from_start (and all
        # later points). The IK can produce a large posture change between two
        # waypoints — most notably the begin_draw→first-approach transition,
        # where the held begin pose and the line-start IK solution sit in
        # different arm configurations (~0.8 rad apart on joint_4/joint_7).
        # Commanded over the normal fraction-of-a-second step that is
        # physically unexecutable, so the arm lags through it and the pen
        # swings off the paper (a vertical line then draws at ~36% coverage).
        # That jump happens pen-UP, so simply giving it enough time fixes it
        # without affecting drawn-line geometry. 0 disables the pass.
        self.declare_parameter('max_joint_speed', 1.2)

        gp = lambda n: self.get_parameter(n).value
        self.begin_draw_joints = np.array(gp('begin_draw_joints'), dtype=float)
        self.pen_offset_m      = float(gp('pen_offset_mm')) / 1000.0
        self.pen_offset_ref_m  = float(gp('pen_offset_ref_mm')) / 1000.0
        self.pen_axis_local    = np.array(gp('pen_axis_local'), dtype=float)
        self.pen_axis_local   /= np.linalg.norm(self.pen_axis_local)
        self.t_move_to_begin   = float(gp('move_to_begin_seconds'))
        self.t_dwell           = float(gp('dwell_seconds'))
        self.wx, self.wy       = gp('workspace_x_mm'), gp('workspace_y_mm')
        self.lift_mm           = float(gp('lift_mm'))
        self.max_workspace_mm  = float(gp('max_workspace_mm'))
        self.draw_tilt         = bool(gp('draw_tilt'))
        self.anchor_x_m        = float(gp('canvas_anchor_x_mm')) / 1000.0
        self.anchor_y_m        = float(gp('canvas_anchor_y_mm')) / 1000.0
        self.tilt_max_deg      = float(gp('tilt_max_deg'))
        # Pen-down tilt above which a draw waypoint is re-solved warm from the
        # last low-tilt pose (kills cold-seed flipped-basin flukes). The
        # perpendicular solution exists ~everywhere, so this just nudges the
        # solver back to it; keep it low but above the genuine edge tilt.
        self.TILT_RETRY_DEG    = 25.0
        self.ds_mm             = float(gp('sample_spacing_mm'))
        self.v_draw            = float(gp('draw_speed_mm_s'))
        self.v_travel          = float(gp('travel_speed_mm_s'))
        self.t_approach        = float(gp('approach_seconds'))
        self.n_approach        = int(gp('approach_samples'))
        self.t_settle          = float(gp('initial_settle_seconds'))
        self.t_stroke_dwell    = max(0.0, float(gp('stroke_dwell_seconds')))
        self.corner_angle_deg  = float(gp('corner_angle_deg'))
        self.t_corner_dwell    = max(0.0, float(gp('corner_dwell_seconds')))
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
        self.table_tilt_x_deg  = float(gp('table_tilt_x_deg'))
        self.table_tilt_y_deg  = float(gp('table_tilt_y_deg'))
        self.velocity_ff       = bool(gp('velocity_ff'))
        self.max_joint_speed   = float(gp('max_joint_speed'))

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
            self._paper_R_persistent = self._table_tilt_R() @ np.array([
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
        # The canvas centre maps to the anchor offset (paper mm), so subtract it
        # before normalising; paper_xy then spans [-wx/2..+wx/2] × [-wy/2..+wy/2].
        norm_x = float((paper_xy_mm[0] - self.anchor_x_m * 1000.0) / self.wx + 0.5)
        norm_y = float((paper_xy_mm[1] - self.anchor_y_m * 1000.0) / self.wy + 0.5)
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
            # Signed clearance (negative = up). The 10 mm floor is applied when
            # z_lift is computed, so just store the raw value here.
            self.lift_mm = float(cfg['lift_mm'])
        if 'pen_offset_mm' in cfg:
            # EE flange → pen tip length (mm). Lets the pendant switch pens of
            # different lengths. Changing it shifts the pen tip (and hence the
            # paper plane), so the begin pose may need re-tuning for the new pen.
            self.pen_offset_m = max(0.0, float(cfg['pen_offset_mm'])) / 1000.0

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
        # Pin the paper to a fixed table point: the begin pose was tuned for
        # `pen_offset_ref_m`, whose pen tip sits at `ref_tip` (the table). If the
        # live pen is longer/shorter, re-solve the begin pose so the ACTUAL pen
        # tip lands at the SAME table point — i.e. a longer pen raises the EE.
        # Without this the pen length cancels out (paper is anchored to the tip)
        # and changing pen_offset_mm has NO effect on the motion. Uses the same
        # pen-tip IK as drawing (position-primary, robust), seeded at q_begin.
        if abs(self.pen_offset_m - self.pen_offset_ref_m) > 1e-4:
            pen_dir_ref = R_begin @ self.pen_axis_local
            ref_tip = T_begin[:3, 3] + self.pen_offset_ref_m * pen_dir_ref
            tool_off = self.pen_offset_m * self.pen_axis_local
            q_adj, _r, _tilt, _c = solve_ik_tip(
                self._chain, ref_tip, R_begin, tool_off, q_begin,
                params={'dq_max': 0.05, 'max_iters': 2000, 'tol_pos': 1e-3,
                        'joint_weights': self.joint_weights, 'null_k': 0.0})
            if _c:
                q_begin = q_adj
                _, T_begin = self._chain.fk(q_begin)
                R_begin = T_begin[:3, :3]
                self.get_logger().info(
                    f'pen-offset z-adjust: pen {self.pen_offset_m*1000:.0f} mm '
                    f'(ref {self.pen_offset_ref_m*1000:.0f}) — begin EE z='
                    f'{T_begin[2,3]*1000:.0f} mm, tip pinned to table')
            else:
                self.get_logger().warn(
                    f'pen-offset z-adjust IK failed (pen '
                    f'{self.pen_offset_m*1000:.0f} mm, err {_r*1000:.1f} mm) — '
                    f'pen length out of reach; using base begin pose')
        # Pen extends from EE along `pen_axis_local` in the EE local frame.
        pen_dir_base     = R_begin @ self.pen_axis_local
        pen_tip_at_begin = T_begin[:3, 3] + self.pen_offset_m * pen_dir_base
        # Keep the live-pen-position broadcast anchor in sync with the pen
        # offset used for THIS drawing — otherwise /pen_canvas_norm is
        # normalised against the startup (launch-default) anchor and the dot
        # drifts whenever pen_offset_mm is changed from the pendant.
        self._pen_anchor_base = pen_tip_at_begin

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
        # Tilt the (base-horizontal) drawing plane to match a non-level table so
        # the pen keeps constant contact across the canvas. Identity when the
        # table_tilt params are 0.
        paper_R = self._table_tilt_R() @ paper_R

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
        # Paper plane sits at the begin-pose pen-tip height (z_paper = 0 in the
        # paper frame); the pen lifts to z_lift above it between strokes. lift_mm
        # is the signed clearance (negative = up); z_lift is the positive travel
        # height, floored at 10 mm so the tip never reaches the table and uncapped
        # above.
        self.z_paper = 0.0
        self.z_lift  = max(10.0, -self.lift_mm)
        # Orientation stays = R_begin throughout. q_draw/q_approach are
        # kept identical so the SLERP in the approach/lift loops is a
        # no-op (orientation already correct from the joint-space move
        # to begin_draw). The pen tilt is set entirely by
        # begin_draw_joints[6] — change that to control how perpendicular
        # the pen sits during drawing.
        self.q_draw     = _R_to_quat_wxyz(R_begin)
        self.q_approach = self.q_draw.copy()

        t_plan_start = time.perf_counter()  # Table 4.1 timing
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

        # Begin/dwell as a pen-UP HOVER at the lift height above the paper
        # origin (matching the end-of-drawing lift pose) instead of resting the
        # pen on the paper at the start. The paper plane and the per-waypoint IK
        # seed stay = q_begin; only the held pose is raised by the lift height.
        hover_tip = pen_tip_at_begin + paper_R @ np.array([0.0, 0.0, self.z_lift / 1000.0])
        tool_off_hover = self.pen_offset_m * self.pen_axis_local
        q_hover, _hr, _ht, _hc = solve_ik_tip(
            self._chain, hover_tip, R_begin, tool_off_hover, q_begin,
            params={'dq_max': 0.05, 'max_iters': 2000, 'tol_pos': 1e-3,
                    'joint_weights': self.joint_weights, 'null_k': 0.0})
        if not _hc:
            self.get_logger().warn(
                f'begin-hover IK failed (err {_hr*1000:.1f} mm) — '
                f'holding at the paper instead')
            q_hover = q_begin

        pt_begin = JointTrajectoryPoint()
        pt_begin.positions = q_hover.tolist()
        pt_begin.time_from_start = self._seconds_to_duration(self.t_move_to_begin)
        traj.points.append(pt_begin)
        point_kinds.append('begin_draw')

        # Phase 2: hold at the hover pose for dwell_seconds.
        t_drawing_start = self.t_move_to_begin + self.t_dwell
        pt_hold = JointTrajectoryPoint()
        pt_hold.positions = q_hover.tolist()
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
        n_high_tilt = 0               # pen-down points left above TILT_RETRY_DEG
        cum_t = t_drawing_start + self.t_settle
        n_unconverged = 0
        max_resid = 0.0
        first_resid = None
        max_tilt_deg = 0.0   # worst pen tilt off perpendicular (tilt mode only)
        unconv_kinds = {}    # {kind: count} of waypoints that didn't converge
        worst_unconv = {'resid': 0.0}
        ik_time_total = 0.0  # Table 4.1 timing — accumulated solve_ik wall time
        tool_offset = self.pen_offset_m * self.pen_axis_local  # tip in EE frame

        for idx, (x, y, z, q_wxyz, dt, _kind) in enumerate(cart_wps):
            # Treat (x, y, z) as paper-frame meters and transform to base
            # using paper_R (which has paper +Z aligned with the pen
            # direction in base). Orientation stays = R_begin throughout
            # drawing — the wrist tilt is set by begin_draw_joints[6] and
            # the IK keeps it there via the heavy joint_7 weight.
            # Paper-frame target (m), re-centred by the canvas anchor offset.
            paper_xyz = np.array([x + self.anchor_x_m, y + self.anchor_y_m, z])
            # ALWAYS seed from q_begin and pull toward q_begin in null space.
            # The prober verified every cell of the drawable region converges
            # from this seed, so each waypoint lands in the same branch.
            # Warm-seeding from q_seed = previous solution let the IK wander
            # into a far-from-q_begin branch once mid-stroke and then "stick"
            # there because q_null_target tracked the bad seed. Re-seeding
            # kills that drift.
            _ik_t0 = time.perf_counter()
            if self.draw_tilt:
                # Control the PEN TIP; hold pen ⟂ paper (R_begin) as a
                # secondary task that yields to position near the edges.
                # COLD-seed from q_begin (the perpendicular begin posture): the
                # solver then prefers the solution closest to perpendicular, so
                # the pen tilts only as much as each target forces (low tilt in
                # the middle, more at the edges). On the rare target the cold
                # basin misses, WARM-retry from the previous solution (a
                # converged, near-perpendicular neighbour) to guarantee
                # convergence without letting the tilt drift.
                tip_params = {'dq_max': 0.05, 'max_iters': 1200, 'tol_pos': 1e-3,
                              'joint_weights': self.joint_weights, 'null_k': 0.0}
                p_tip_des = pen_tip_at_begin + paper_R @ paper_xyz
                # Pure cold-seed from the perpendicular begin posture. The
                # offline reach map shows this converges at low tilt across the
                # whole canvas, so no warm fallback is needed (and a warm seed
                # would let a stray tilted pose cascade down the stroke).
                q_sol, resid, tilt_rad, conv = solve_ik_tip(
                    self._chain, p_tip_des, R_begin, tool_offset, q_begin,
                    params=tip_params)
                tdeg = np.degrees(tilt_rad)
                if _kind == 'draw' and conv and tdeg > self.TILT_RETRY_DEG:
                    n_high_tilt += 1
                # Only pen-DOWN ('draw') tilt matters for line quality; pen-up
                # travel/lift/approach points can tilt freely (off the paper).
                if _kind == 'draw':
                    max_tilt_deg = max(max_tilt_deg, tdeg)
            else:
                # Perpendicular everywhere: control the EE origin, fixed
                # orientation = R_begin.
                ee_pos_base = T_begin[:3, 3] + paper_R @ paper_xyz
                T_des = np.eye(4)
                T_des[:3, :3] = R_begin
                T_des[:3,  3] = ee_pos_base
                ik_params_step = {**ik_params, 'q_null_target': q_begin}
                q_sol, resid, conv = solve_ik(
                    self._chain, T_des, q_begin,
                    use_null_space=True, params=ik_params_step,
                )
            ik_time_total += time.perf_counter() - _ik_t0
            point_kinds.append(_kind)

            if idx == 0:
                first_resid = resid
                # Detailed first-call diagnostic so unreachable targets are
                # immediately obvious from the log. In tilt mode the controlled
                # point is the pen tip; in perpendicular mode it's the EE origin.
                _, T_final = self._chain.fk(q_sol)
                if self.draw_tilt:
                    p_ach = T_final[:3, 3] + T_final[:3, :3] @ tool_offset
                    p_tgt = p_tip_des
                else:
                    p_ach = T_final[:3, 3]
                    p_tgt = ee_pos_base
                e_p = p_tgt - p_ach
                e_r = rot_error(T_final[:3, :3], R_begin)
                self.get_logger().info(
                    f'IK[0]: pos_err={np.linalg.norm(e_p)*1000:.1f} mm, '
                    f'rot_err={np.linalg.norm(e_r):.3f} rad; '
                    f'{"pen tip" if self.draw_tilt else "EE"} ended at '
                    f'({p_ach[0]:+.3f}, {p_ach[1]:+.3f}, {p_ach[2]:+.3f}) m; '
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
                unconv_kinds[_kind] = unconv_kinds.get(_kind, 0) + 1
                worst_unconv['resid'] = max(worst_unconv['resid'], resid)
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

        # Re-time segments that would demand an unexecutable joint speed
        # (the begin_draw→first-approach posture jump, mainly). Stretches
        # time_from_start; must run BEFORE _fill_velocities so the velocities
        # reflect the final timing.
        if self.max_joint_speed > 0.0:
            self._retime_for_joint_speed(traj, self.max_joint_speed)

        # Velocity feedforward: fill pt.velocities from a central finite
        # difference of positions over the timestamps. The velocity-mode JTC
        # multiplies these by ff_velocity_scale (=1.0) and feeds them forward,
        # so the proportional gain only has to correct residual error instead
        # of driving the whole motion. Endpoints get zero velocity (the
        # trajectory starts and ends at rest). Without this the points are
        # positions-only and the pen badly under-tracks the sweep.
        if self.velocity_ff:
            self._fill_velocities(traj)

        if self.log_joint_deltas:
            self._dump_trajectory(traj, point_kinds)

        self._traj_pub.publish(traj)
        self.get_logger().info(
            f'→ JointTrajectory: {len(traj.points)} points, '
            f'duration={cum_t:.1f}s, first_residual={first_resid:.2e}, '
            f'max_residual={max_resid:.2e}, unconverged={n_unconverged}'
        )
        if n_unconverged:
            self.get_logger().warn(
                f'unconverged by kind: {unconv_kinds} '
                f'(worst residual {worst_unconv["resid"]:.2e})')
        if self.draw_tilt:
            msg = (f'tilt mode: max pen tilt off perpendicular = {max_tilt_deg:.1f} deg'
                   f'; pen-down points above {self.TILT_RETRY_DEG:.0f} deg: {n_high_tilt}')
            if self.tilt_max_deg > 0.0 and max_tilt_deg > self.tilt_max_deg:
                self.get_logger().warn(
                    msg + f' (exceeds tilt_max_deg={self.tilt_max_deg:.0f}; '
                    f'shrink workspace_x/y_mm or re-centre canvas_anchor_*)')
            else:
                self.get_logger().info(msg)
        # Table 4.1 timing — read these off the console after a drawing run.
        n_ik = len(cart_wps)
        plan_ms = (time.perf_counter() - t_plan_start) * 1e3
        self.get_logger().info(
            f'[TIMING] batch generation {plan_ms:.1f} ms total; '
            f'IK {n_ik} waypoints in {ik_time_total * 1e3:.1f} ms '
            f'({ik_time_total / max(1, n_ik) * 1e3:.2f} ms/waypoint avg); '
            f'controller dispatch @ 100 Hz (arm_robot_controllers.yaml update_rate)'
        )

    # ── Planning helpers ───────────────────────────────────────────────────

    def _retime_for_joint_speed(self, traj: JointTrajectory,
                                max_speed: float) -> None:
        """Stretch any segment whose required joint speed exceeds max_speed
        (rad/s) by pushing out its time_from_start and every later point's.
        Leaves feasible segments untouched. Logs the total time added."""
        pts = traj.points
        if len(pts) < 2:
            return
        added = 0.0
        n_stretched = 0
        worst = (0, 0.0, 0.0)  # (idx, required_speed, dq)
        for i in range(1, len(pts)):
            dq = np.abs(np.asarray(pts[i].positions)
                        - np.asarray(pts[i - 1].positions))
            dq_max = float(dq.max()) if dq.size else 0.0
            t_prev = pts[i - 1].time_from_start.sec + \
                pts[i - 1].time_from_start.nanosec * 1e-9
            t_cur = pts[i].time_from_start.sec + \
                pts[i].time_from_start.nanosec * 1e-9
            dt = t_cur - t_prev
            dt_min = dq_max / max_speed
            if dt_min > dt + 1e-6:
                extra = dt_min - dt
                added += extra
                n_stretched += 1
                speed = dq_max / dt if dt > 1e-9 else float('inf')
                if speed > worst[1]:
                    worst = (i, speed, dq_max)
                # push this point and all later points out by `extra`
                for k in range(i, len(pts)):
                    t = pts[k].time_from_start.sec + \
                        pts[k].time_from_start.nanosec * 1e-9 + extra
                    pts[k].time_from_start = self._seconds_to_duration(t)
        if n_stretched:
            self.get_logger().info(
                f'retimed {n_stretched} segment(s) to cap joint speed at '
                f'{max_speed:.1f} rad/s (+{added:.1f}s total; worst was '
                f'{worst[1]:.1f} rad/s, Δq={worst[2]:.2f} rad at point {worst[0]})'
            )

    @staticmethod
    def _fill_velocities(traj: JointTrajectory) -> None:
        """Set pt.velocities on every point via a central finite difference of
        positions over time_from_start. Endpoints are clamped to zero so the
        motion starts/ends at rest. No-op for trajectories with < 3 points."""
        pts = traj.points
        n = len(pts)
        if n < 2:
            for pt in pts:
                pt.velocities = [0.0] * len(pt.positions)
            return
        t = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                      for p in pts])
        q = np.array([p.positions for p in pts], dtype=float)  # (n, ndof)
        v = np.zeros_like(q)
        # interior points: central difference (n-1 .. 1)
        for i in range(1, n - 1):
            dt = t[i + 1] - t[i - 1]
            if dt > 1e-9:
                v[i] = (q[i + 1] - q[i - 1]) / dt
        # endpoints stay at rest (start at q_start, end after lift)
        for i, pt in enumerate(pts):
            pt.velocities = v[i].tolist()

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

    def _table_tilt_R(self):
        """Base-frame rotation that tilts the otherwise-horizontal drawing plane
        to match a non-level table: Rx(table_tilt_x) @ Ry(table_tilt_y). Identity
        when both tilts are 0 (the default)."""
        tx = np.radians(self.table_tilt_x_deg)
        ty = np.radians(self.table_tilt_y_deg)
        cx, sxx = float(np.cos(tx)), float(np.sin(tx))
        cy, syy = float(np.cos(ty)), float(np.sin(ty))
        Rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sxx], [0.0, sxx, cx]])
        Ry = np.array([[cy, 0.0, syy], [0.0, 1.0, 0.0], [-syy, 0.0, cy]])
        return Rx @ Ry

    def _detect_corners(self, xs, ys):
        """Indices of sharp-corner vertices in a polyline (mm coords).

        A vertex is a corner when the turn angle (deviation from straight)
        between its incoming and outgoing chord exceeds corner_angle_deg.
        Sub-millimetre jitter segments are ignored so noisy freehand curves
        don't register false corners. Returns a sorted list of interior
        indices (empty if corner splitting is disabled or the stroke is short).
        """
        n = len(xs)
        if n < 3 or self.corner_angle_deg <= 0.0:
            return []
        thr = np.radians(self.corner_angle_deg)
        min_seg = 1.0  # mm — ignore jitter shorter than this on either side
        corners = []
        for i in range(1, n - 1):
            v1x, v1y = xs[i] - xs[i - 1], ys[i] - ys[i - 1]
            v2x, v2y = xs[i + 1] - xs[i], ys[i + 1] - ys[i]
            n1 = float(np.hypot(v1x, v1y))
            n2 = float(np.hypot(v2x, v2y))
            if n1 < min_seg or n2 < min_seg:
                continue
            cosang = (v1x * v2x + v1y * v2y) / (n1 * n2)
            turn = float(np.arccos(np.clip(cosang, -1.0, 1.0)))
            if turn > thr:
                corners.append(i)
        return corners

    def _resample_segment(self, xs, ys):
        """Resample one polyline segment at uniform chord spacing (ds_mm).

        Cubic spline for >=4 points (smooth curves), linear otherwise. Returns
        (xs_s, ys_s). The original endpoints are preserved exactly, so splitting
        a stroke here and stitching segments keeps corners sharp.
        """
        if len(xs) < 2:
            return np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)
        dx = np.diff(xs)
        dy = np.diff(ys)
        seg = np.hypot(dx, dy)
        u = np.concatenate(([0.0], np.cumsum(seg)))
        L = float(u[-1])
        if L < 1e-6:
            return np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)
        n_samples = max(2, int(round(L / self.ds_mm)) + 1)
        u_new = np.linspace(0.0, L, n_samples)
        if len(xs) >= 4:
            cs_x = CubicSpline(u, xs, bc_type='natural')
            cs_y = CubicSpline(u, ys, bc_type='natural')
            return cs_x(u_new), cs_y(u_new)
        return np.interp(u_new, u, xs), np.interp(u_new, u, ys)

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

            # Corner-aware resampling. A single cubic spline through the whole
            # stroke rounds genuine corners (a rectangle's 90° turns) into arcs.
            # Split the stroke at detected sharp corners and resample each
            # near-straight/curved segment INDEPENDENTLY, keeping the corner as
            # an exact shared vertex. corner_set holds indices into xs_s/ys_s
            # that are corner vertices (used to optionally hold there for a crisp
            # stop). With corner_angle_deg=0 (or no corners) this collapses to a
            # single segment = the original whole-stroke spline.
            corner_set: set[int] = set()
            if len(xs) < 2:
                xs_s, ys_s = xs.copy(), ys.copy()
            else:
                corners = self._detect_corners(xs, ys)
                bounds = [0] + corners + [len(xs) - 1]
                xparts, yparts = [], []
                running = 0
                for k in range(len(bounds) - 1):
                    a, b = bounds[k], bounds[k + 1]
                    rx, ry = self._resample_segment(xs[a:b + 1], ys[a:b + 1])
                    if k > 0:
                        rx, ry = rx[1:], ry[1:]   # drop the junction duplicate
                    xparts.append(rx)
                    yparts.append(ry)
                    running += len(rx)
                    # The vertex at the END of this segment (= bounds[k+1]) is a
                    # corner for every segment but the last.
                    if k < len(bounds) - 2 and len(rx) > 0:
                        corner_set.add(running - 1)
                xs_s = np.concatenate(xparts) if xparts else xs.copy()
                ys_s = np.concatenate(yparts) if yparts else ys.copy()

            # Clamp resampled points to the canvas box: a cubic spline through
            # sharp corners can overshoot past the edges, pushing waypoints
            # outside the reachable drawable area (IK miss / extreme tilt). The
            # canvas is sized to be reachable, so clamping keeps every point in.
            xs_s = np.clip(xs_s, self.ox, self.ox + self.wx)
            ys_s = np.clip(ys_s, self.oy, self.oy + self.wy)

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

            # 2b. Pen-down hold at the stroke start: the pen is now on the paper
            #     at (x0, y0); hold here before dragging so the start dot is
            #     clean and the user can see the pen settle. Zero-motion waypoint
            #     (same pose) → the arm stops fully during the hold.
            if self.t_stroke_dwell > 0.0:
                wps.append((
                    x0_mm / 1000.0,
                    y0_mm / 1000.0,
                    self.z_paper / 1000.0,
                    self.q_draw,
                    self.t_stroke_dwell,
                    'dwell',
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
                # Optional crisp-stop hold AT the corner vertex (zero-motion).
                if i in corner_set and self.t_corner_dwell > 0.0:
                    wps.append((
                        float(xs_s[i]) / 1000.0,
                        float(ys_s[i]) / 1000.0,
                        self.z_paper   / 1000.0,
                        self.q_draw,
                        self.t_corner_dwell,
                        'corner',
                    ))

            # 3b. Pen-down hold at the stroke end: the pen is still on the paper
            #     at (xN, yN); hold here before lifting (mirrors the start hold).
            if self.t_stroke_dwell > 0.0:
                wps.append((
                    xN_mm / 1000.0,
                    yN_mm / 1000.0,
                    self.z_paper / 1000.0,
                    self.q_draw,
                    self.t_stroke_dwell,
                    'dwell',
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
