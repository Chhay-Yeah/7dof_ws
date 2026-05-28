#!/usr/bin/env python3
"""
drawing_trajectory_planner.py

Subscribes:
  /drawing/strokes    (std_msgs/String, JSON)
  /joint_states       (sensor_msgs/JointState) — current pose for the lead-in
  /robot_description  (std_msgs/String, latched) — URDF for the FK chain
Publishes:
  /cartesian_path     (geometry_msgs/PoseArray, base frame)

Live sibling of drawing_batch_planner: it builds the SAME reachable paper
frame, but instead of running offline IK and emitting one JointTrajectory it
publishes the Cartesian waypoints on /cartesian_path for drawing_executor_node
to stream through the live IK node (ik_arm_v3).

Why this rewrite: the previous version commanded an absolute base-frame
workspace (200x150 mm at x=300, z=0) with the pen forced straight down
(q=[0,1,0,0]). That pose is almost entirely outside the arm's reach — only
the dead centre solved, and even that pinned joint_2 at its limit; the corners
missed by 45-95 mm. The IK saturated joints and the drawing came out wrong.

The paper is now anchored on the pen tip at a reachable begin_draw posture and
the drawing orientation is that posture's natural EE orientation (the same
trick the batch planner uses), so every waypoint in the workspace is reachable.
"""
import json
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from scipy.interpolate import CubicSpline
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose

from arm_bot.ik_lib import UrdfChain


def _R_to_quat_wxyz(R):
    """3x3 rotation matrix -> quaternion (w, x, y, z). Shepperd's method."""
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


class DrawingTrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('drawing_trajectory_planner')

        # Begin-draw posture + paper-frame params — defaults match the batch
        # planner's tuned block (pendant_backend.launch.py). Keep in sync.
        self.declare_parameter('begin_draw_joints',
                               [0.0, -0.7, 0.0, 1.4, 0.01, 0.0, 1.0])
        self.declare_parameter('pen_offset_mm',     100.0)
        self.declare_parameter('pen_axis_local',    [1.0, 0.0, 0.0])
        self.declare_parameter('workspace_x_mm',    40.0)
        self.declare_parameter('workspace_y_mm',    40.0)
        self.declare_parameter('lift_mm',           10.0)
        self.declare_parameter('sample_spacing_mm', 2.0)
        self.declare_parameter('paper_rotation_deg', 270)
        self.declare_parameter('paper_mirror_x',    False)
        # Number of begin-pose repeats prepended after the lead-in slew, so
        # the live IK settles at begin_draw before the first stroke. The
        # executor pops one per tick, so this is a dwell of settle_points/rate.
        self.declare_parameter('settle_points',     20)
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('tip_link',  'ee')
        self.declare_parameter('frame_id',  'base_link')

        gp = lambda n: self.get_parameter(n).value
        self.begin_draw_joints = np.array(gp('begin_draw_joints'), dtype=float)
        self.pen_offset_m   = float(gp('pen_offset_mm')) / 1000.0
        self.pen_axis_local = np.array(gp('pen_axis_local'), dtype=float)
        self.pen_axis_local /= np.linalg.norm(self.pen_axis_local)
        self.wx = float(gp('workspace_x_mm'))
        self.wy = float(gp('workspace_y_mm'))
        self.lift_mm = float(gp('lift_mm'))
        self.ds_mm = float(gp('sample_spacing_mm'))
        self.paper_rotation_deg = int(gp('paper_rotation_deg'))
        self.paper_mirror_x = bool(gp('paper_mirror_x'))
        self.settle_points = int(gp('settle_points'))
        self.base_link = gp('base_link')
        self.tip_link  = gp('tip_link')
        self.frame_id  = gp('frame_id')

        self._chain: Optional[UrdfChain] = None
        self._q_current: Optional[np.ndarray] = None
        self._joint_index = None

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(String, '/robot_description', self._cb_urdf, latched)
        self.create_subscription(JointState, '/joint_states', self._cb_joints, 30)
        self.create_subscription(String, '/drawing/strokes', self.on_drawing, 10)
        self.path_pub = self.create_publisher(PoseArray, '/cartesian_path', 10)

        self.get_logger().info(
            f'drawing_trajectory_planner ready — chain {self.base_link} -> {self.tip_link}')

    def _cb_urdf(self, msg: String):
        if self._chain is not None:
            return
        try:
            self._chain = UrdfChain(msg.data, self.base_link, self.tip_link)
        except Exception as e:
            self.get_logger().error(f'URDF parse failed: {e}')
            return
        self.get_logger().info(
            f'URDF loaded: {self._chain.n} DoF — joints: {self._chain.joint_names}')

    def _cb_joints(self, msg: JointState):
        if self._chain is None:
            return
        if self._joint_index is None:
            try:
                self._joint_index = [msg.name.index(n)
                                     for n in self._chain.joint_names]
            except ValueError:
                return
        self._q_current = np.array([msg.position[i] for i in self._joint_index],
                                   dtype=float)

    def on_drawing(self, msg: String):
        if self._chain is None:
            self.get_logger().warn('No URDF yet — cannot plan')
            return
        if self._q_current is None:
            self.get_logger().warn('No /joint_states yet — cannot plan')
            return
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Bad JSON: {e}')
            return
        if not data.get('strokes') or all(not s.get('points')
                                          for s in data['strokes']):
            self.get_logger().warn('Empty drawing — nothing to plan')
            return
        if self.begin_draw_joints.shape[0] != self._chain.n:
            self.get_logger().error(
                f'begin_draw_joints has {self.begin_draw_joints.shape[0]} '
                f'entries; URDF chain has {self._chain.n} DoF')
            return

        q_begin = np.clip(self.begin_draw_joints, self._chain.q_min,
                          self._chain.q_max)
        _, T_begin = self._chain.fk(q_begin)
        R_begin = T_begin[:3, :3]
        t_begin = T_begin[:3, 3]
        q_draw = _R_to_quat_wxyz(R_begin)

        theta = np.radians(self.paper_rotation_deg)
        c, s = float(np.cos(theta)), float(np.sin(theta))
        sx = -1.0 if self.paper_mirror_x else 1.0
        paper_R = np.array([
            [c * sx, -s,  0.0],
            [s * sx,  c,  0.0],
            [0.0,     0.0, 1.0],
        ])

        # Paper-frame (meters) waypoints: approach -> draw -> lift -> travel.
        paper_wps = self._build_paper_waypoints(data)
        if not paper_wps:
            self.get_logger().warn('Planner produced no waypoints')
            return

        poses = []

        # Lead-in: current EE pose -> begin pose, then hold at begin to settle.
        # The executor interpolates current->begin into small steps so the
        # live IK slews there smoothly before drawing.
        _, T_cur = self._chain.fk(self._q_current)
        poses.append(self._make_pose(T_cur[:3, 3], _R_to_quat_wxyz(T_cur[:3, :3])))
        for _ in range(max(1, self.settle_points)):
            poses.append(self._make_pose(t_begin, q_draw))

        # Drawing waypoints, transformed paper -> base.
        for (px, py, pz) in paper_wps:
            base = t_begin + paper_R @ np.array([px, py, pz])
            poses.append(self._make_pose(base, q_draw))

        out = PoseArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.poses = poses
        self.path_pub.publish(out)

        pen_dir = R_begin @ self.pen_axis_local
        self.get_logger().info(
            f'Published path: {len(poses)} poses from {len(data["strokes"])} '
            f'strokes; begin EE=({t_begin[0]:+.3f},{t_begin[1]:+.3f},'
            f'{t_begin[2]:+.3f}) pen_dir=({pen_dir[0]:+.2f},{pen_dir[1]:+.2f},'
            f'{pen_dir[2]:+.2f}) box=±{self.wx/2:.0f}x±{self.wy/2:.0f}mm')

    def _build_paper_waypoints(self, data):
        """Return list of (x_m, y_m, z_m) in the paper frame. z=0 is the paper
        plane; +lift_mm is pen-up. Strokes are spline-resampled at uniform
        chord spacing; between strokes the pen travels at lift height."""
        cw = data['canvas']['width']
        ch = data['canvas']['height']
        sx = self.wx / cw
        sy = self.wy / ch
        ox = -self.wx / 2.0
        oy = -self.wy / 2.0
        z_paper = 0.0
        z_lift = self.lift_mm

        wps = []
        prev_lift_xy = None

        for stroke in data['strokes']:
            pts = stroke.get('points', [])
            if not pts:
                continue

            xs = np.array([ox + p['x'] * sx for p in pts], dtype=float)
            ys = np.array([oy + (ch - p['y']) * sy for p in pts], dtype=float)

            keep = [0]
            for i in range(1, len(xs)):
                if (xs[i] - xs[keep[-1]])**2 + (ys[i] - ys[keep[-1]])**2 > 1e-8:
                    keep.append(i)
            xs, ys = xs[keep], ys[keep]
            if len(xs) < 2:
                xs_s, ys_s = xs.copy(), ys.copy()
            else:
                seg = np.hypot(np.diff(xs), np.diff(ys))
                u = np.concatenate(([0.0], np.cumsum(seg)))
                L = float(u[-1])
                if L < 1e-6:
                    xs_s, ys_s = xs.copy(), ys.copy()
                else:
                    n_samples = max(2, int(round(L / self.ds_mm)) + 1)
                    u_new = np.linspace(0.0, L, n_samples)
                    if len(xs) >= 4:
                        xs_s = CubicSpline(u, xs, bc_type='natural')(u_new)
                        ys_s = CubicSpline(u, ys, bc_type='natural')(u_new)
                    else:
                        xs_s = np.interp(u_new, u, xs)
                        ys_s = np.interp(u_new, u, ys)

            x0, y0 = float(xs_s[0]), float(ys_s[0])
            xN, yN = float(xs_s[-1]), float(ys_s[-1])

            # Travel from previous lift to over this stroke's start (lift height)
            if prev_lift_xy is not None:
                wps.append((prev_lift_xy[0] / 1000.0,
                            prev_lift_xy[1] / 1000.0, z_lift / 1000.0))
                wps.append((x0 / 1000.0, y0 / 1000.0, z_lift / 1000.0))

            # Approach: lift -> paper over the start point
            wps.append((x0 / 1000.0, y0 / 1000.0, z_lift / 1000.0))
            wps.append((x0 / 1000.0, y0 / 1000.0, z_paper / 1000.0))

            # Draw
            for i in range(len(xs_s)):
                wps.append((float(xs_s[i]) / 1000.0,
                            float(ys_s[i]) / 1000.0, z_paper / 1000.0))

            # Lift at the end point
            wps.append((xN / 1000.0, yN / 1000.0, z_lift / 1000.0))
            prev_lift_xy = (xN, yN)

        return wps

    @staticmethod
    def _make_pose(pos_m, q_wxyz):
        pose = Pose()
        pose.position.x = float(pos_m[0])
        pose.position.y = float(pos_m[1])
        pose.position.z = float(pos_m[2])
        pose.orientation.w = float(q_wxyz[0])
        pose.orientation.x = float(q_wxyz[1])
        pose.orientation.y = float(q_wxyz[2])
        pose.orientation.z = float(q_wxyz[3])
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = DrawingTrajectoryPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
