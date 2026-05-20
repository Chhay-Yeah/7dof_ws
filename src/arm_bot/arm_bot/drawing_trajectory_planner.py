#!/usr/bin/env python3
"""
drawing_trajectory_planner.py

Subscribes:  /drawing/strokes  (std_msgs/String, JSON-encoded)
Publishes:   /cartesian_path   (geometry_msgs/PoseArray)

Pipeline:
  canvas pixels  →  workspace mm  →  add Z-hops between strokes
                 →  attach orientations via SLERP at transitions
                 →  publish PoseArray for IK node
"""

import json
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose


class DrawingTrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('drawing_trajectory_planner')

        # ── Parameters (declare so they can be set from launch/CLI) ────────
        self.declare_parameter('workspace_x_mm',  200.0)
        self.declare_parameter('workspace_y_mm',  150.0)
        self.declare_parameter('workspace_origin_x_mm', 300.0)  # paper offset in robot frame
        self.declare_parameter('workspace_origin_y_mm', -75.0)
        self.declare_parameter('z_paper_mm', 0.0)               # pen touching paper
        self.declare_parameter('z_lift_mm',  30.0)              # pen-up height
        self.declare_parameter('slerp_samples', 5)              # samples per orientation transition
        self.declare_parameter('frame_id', 'base_link')

        gp = lambda n: self.get_parameter(n).value
        self.wx, self.wy   = gp('workspace_x_mm'),  gp('workspace_y_mm')
        self.ox, self.oy   = gp('workspace_origin_x_mm'), gp('workspace_origin_y_mm')
        self.z_paper       = gp('z_paper_mm')
        self.z_lift        = gp('z_lift_mm')
        self.n_slerp       = int(gp('slerp_samples'))
        self.frame_id      = gp('frame_id')

        # ── Orientation references (quaternions [w, x, y, z]) ──────────────
        # Drawing: pen perpendicular to paper (180° about X → tool-Z points down)
        self.q_draw     = np.array([0.0, 1.0, 0.0, 0.0])
        # Approach: slight tilt so we don't crash straight down — adjust to taste.
        # Here: 170° about X (10° tilt forward). Set equal to q_draw to disable tilt.
        self.q_approach = self._axis_angle_quat([1.0, 0.0, 0.0], np.deg2rad(170.0))

        # ── ROS interfaces ──────────────────────────────────────────────────
        self.create_subscription(String, '/drawing/strokes', self.on_drawing, 10)
        self.path_pub = self.create_publisher(PoseArray, '/cartesian_path', 10)

        self.get_logger().info('Drawing trajectory planner ready')

    # ───────────────────────────────────────────────────────────────────────
    # Subscriber callback
    # ───────────────────────────────────────────────────────────────────────
    def on_drawing(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Bad JSON: {e}')
            return

        if not data.get('strokes') or all(
                not s['points'] for s in data['strokes']):
            self.get_logger().warn('Empty drawing — nothing to plan')
            return

        poses = self._plan_path(data)
        if not poses:
            return

        out = PoseArray()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.poses           = poses
        self.path_pub.publish(out)
        self.get_logger().info(
            f'Published path: {len(poses)} poses from {len(data["strokes"])} strokes')

    # ───────────────────────────────────────────────────────────────────────
    # Planning: canvas → workspace → poses w/ SLERP
    # ───────────────────────────────────────────────────────────────────────
    def _plan_path(self, data):
        cw = data['canvas']['width']
        ch = data['canvas']['height']
        sx = self.wx / cw                # px → mm scale (X)
        sy = self.wy / ch                # px → mm scale (Y)

        poses = []
        for stroke in data['strokes']:
            pts = stroke['points']
            if not pts:
                continue

            # Canvas → workspace mm (flip Y: canvas Y points down, robot Y points up)
            wp = [(self.ox + p['x'] * sx,
                   self.oy + (ch - p['y']) * sy) for p in pts]

            x0, y0 = wp[0]
            xN, yN = wp[-1]

            # 1) Approach: hover over first point, SLERP approach → draw orientation
            for q in self._slerp_seq(self.q_approach, self.q_draw, self.n_slerp):
                poses.append(self._make_pose(x0, y0, self.z_lift, q))

            # 2) Drawing pass: pen at paper, constant draw orientation
            #    (this is where future tangent-aligned orientation would go)
            for (X, Y) in wp:
                poses.append(self._make_pose(X, Y, self.z_paper, self.q_draw))

            # 3) Lift: SLERP draw → approach orientation as pen lifts
            for q in self._slerp_seq(self.q_draw, self.q_approach, self.n_slerp):
                poses.append(self._make_pose(xN, yN, self.z_lift, q))

        return poses

    # ───────────────────────────────────────────────────────────────────────
    # SLERP — Shoemake (1985)
    # ───────────────────────────────────────────────────────────────────────
    @staticmethod
    def _slerp(q0, q1, t):
        q0 = q0 / np.linalg.norm(q0)
        q1 = q1 / np.linalg.norm(q1)
        dot = float(np.dot(q0, q1))
        # Take shorter arc
        if dot < 0.0:
            q1, dot = -q1, -dot
        # Near-parallel: linear interp + renormalize (avoids sin(0) blow-up)
        if dot > 0.9995:
            r = q0 + t * (q1 - q0)
            return r / np.linalg.norm(r)
        theta_0  = np.arccos(dot)
        sin_t0   = np.sin(theta_0)
        theta    = theta_0 * t
        s0 = np.cos(theta) - dot * np.sin(theta) / sin_t0
        s1 = np.sin(theta) / sin_t0
        return s0 * q0 + s1 * q1

    def _slerp_seq(self, q0, q1, n):
        return [self._slerp(q0, q1, t) for t in np.linspace(0.0, 1.0, n)]

    @staticmethod
    def _axis_angle_quat(axis, angle_rad):
        axis = np.array(axis, dtype=float)
        axis = axis / np.linalg.norm(axis)
        half = angle_rad * 0.5
        return np.array([np.cos(half),
                         axis[0] * np.sin(half),
                         axis[1] * np.sin(half),
                         axis[2] * np.sin(half)])

    # ───────────────────────────────────────────────────────────────────────
    @staticmethod
    def _make_pose(x_mm, y_mm, z_mm, q):
        pose = Pose()
        pose.position.x = float(x_mm) / 1000.0   # ROS convention: meters
        pose.position.y = float(y_mm) / 1000.0
        pose.position.z = float(z_mm) / 1000.0
        pose.orientation.w = float(q[0])
        pose.orientation.x = float(q[1])
        pose.orientation.y = float(q[2])
        pose.orientation.z = float(q[3])
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