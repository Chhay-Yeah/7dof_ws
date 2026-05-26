#!/usr/bin/env python3
"""
workspace_prober.py

One-shot probe of the orientation-constrained reachable workspace around
a given begin_draw pose. Samples a square grid of XY positions in the
paper frame (paper = plane perpendicular to the pen at begin_draw) and
tries to solve IK at each one. Prints an ASCII map and a bounding-box
summary so you can pick a workspace_x/y_mm that fits inside the
reachable region.

Usage:
  Start anything that publishes /robot_description first (e.g.
  gazebo.launch.py), then:

    ros2 run arm_bot workspace_prober.py \\
      --ros-args -p begin_draw_joints:='[0.0, 1.187, 0.0, -1.0, 0.0, -0.2, -1.0]' \\
                 -p sample_step_mm:=10.0 \\
                 -p sample_radius_mm:=100.0

The probe runs once when the URDF arrives and then stays idle. Ctrl+C
to exit.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String

from arm_bot.ik_lib import UrdfChain, solve_ik


class WorkspaceProber(Node):
    def __init__(self):
        super().__init__('workspace_prober')

        # Defaults match the tuned config in draw_and_execute_batch.launch.py
        # so `ros2 run arm_bot workspace_prober.py` with no args probes the
        # actual planner pose. Override any of these with `-p NAME:=VALUE`.
        self.declare_parameter('begin_draw_joints',
                               [0.0, -0.7, 0.0, 1.4, 0.4, 0.0, 0.4])
        self.declare_parameter('base_link',         'base_link')
        self.declare_parameter('tip_link',          'ee')
        self.declare_parameter('sample_step_mm',    10.0)
        self.declare_parameter('sample_radius_mm',  80.0)
        self.declare_parameter('lift_test_mm',      10.0)
        self.declare_parameter('joint_weights',
                               [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('null_k',            2.0)
        self.declare_parameter('pen_axis_local',    [1.0, 0.0, 0.0])
        self.declare_parameter('pen_offset_mm',     100.0)

        gp = lambda n: self.get_parameter(n).value
        self.q_begin   = np.array(gp('begin_draw_joints'), dtype=float)
        self.base_link = gp('base_link')
        self.tip_link  = gp('tip_link')
        self.step      = float(gp('sample_step_mm'))
        self.radius    = float(gp('sample_radius_mm'))
        self.lift_mm   = float(gp('lift_test_mm'))
        self.weights        = list(gp('joint_weights'))
        self.null_k         = float(gp('null_k'))
        self.pen_axis_local = np.array(gp('pen_axis_local'), dtype=float)
        self.pen_axis_local /= np.linalg.norm(self.pen_axis_local)
        self.pen_offset_m   = float(gp('pen_offset_mm')) / 1000.0

        self._done = False

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String, '/robot_description', self._cb_urdf, latched
        )
        self.get_logger().info(
            'workspace_prober waiting for /robot_description ...'
        )

    def _cb_urdf(self, msg: String):
        if self._done:
            return
        try:
            chain = UrdfChain(msg.data, self.base_link, self.tip_link)
        except Exception as e:
            self.get_logger().error(f'URDF parse failed: {e}')
            return
        self.get_logger().info(f'URDF loaded: {chain.n} DoF')
        self._probe(chain)
        self._done = True

    def _probe(self, chain: UrdfChain) -> None:
        if self.q_begin.shape[0] != chain.n:
            self.get_logger().error(
                f'begin_draw_joints has {self.q_begin.shape[0]} entries; '
                f'chain has {chain.n} DoF'
            )
            return

        q_begin = np.minimum(np.maximum(self.q_begin, chain.q_min),
                             chain.q_max)
        _, T_begin = chain.fk(q_begin)
        R_begin = T_begin[:3, :3]
        pen_dir = R_begin @ self.pen_axis_local

        # Horizontal-paper convention: paper_R = identity, drawing
        # waypoints translate the EE in base XY at constant base z =
        # T_begin.z, lift moves +base Z. Mirrors the planner.
        paper_R = np.eye(3)

        self.get_logger().info(
            f'begin_draw EE at ({T_begin[0,3]:+.3f}, {T_begin[1,3]:+.3f}, '
            f'{T_begin[2,3]:+.3f}) m'
        )
        # Show all three EE-local axes in base frame so you can identify
        # which direction physically corresponds to "pen down". Look for
        # the column with z ≈ -1 — that's the axis to set as the pen
        # direction in the planner.
        self.get_logger().info(
            f'EE local +X in base: ({R_begin[0,0]:+.3f}, {R_begin[1,0]:+.3f}, '
            f'{R_begin[2,0]:+.3f})'
        )
        self.get_logger().info(
            f'EE local +Y in base: ({R_begin[0,1]:+.3f}, {R_begin[1,1]:+.3f}, '
            f'{R_begin[2,1]:+.3f})'
        )
        self.get_logger().info(
            f'EE local +Z in base: ({R_begin[0,2]:+.3f}, {R_begin[1,2]:+.3f}, '
            f'{R_begin[2,2]:+.3f})'
        )
        self.get_logger().info(
            f'pen_axis_local = {self.pen_axis_local.tolist()} → '
            f'pen direction in base: ({pen_dir[0]:+.3f}, {pen_dir[1]:+.3f}, '
            f'{pen_dir[2]:+.3f})  '
            f'(paper plane is perpendicular to this)'
        )
        self.get_logger().info(
            f'sampling paper-frame XY: '
            f'±{self.radius:.0f} mm in steps of {self.step:.0f} mm; '
            f'lift test at {self.lift_mm:.0f} mm above paper; '
            f'joint_weights={self.weights}, null_k={self.null_k}'
        )

        ik_params = dict(
            dq_max=0.15,
            max_iters=600,
            null_k=self.null_k,
            q_null_target=q_begin,
            joint_weights=self.weights,
            # Drawing-realistic tolerances. Default 1e-5 m / 1e-4 rad is
            # way too tight for DLS — most targets fail to converge purely
            # because the residual lingers at 1e-4 m even when reachable.
            tol_pos=1e-3,
            tol_rot=1e-2,
        )

        n = int(round(self.radius / self.step)) * 2 + 1
        coords = np.linspace(-self.radius, +self.radius, n)
        results_paper = np.zeros((n, n), dtype=bool)
        results_lift  = np.zeros((n, n), dtype=bool)
        resids_paper  = np.full((n, n), np.nan)

        # Collect FK-computed EE and pen-tip positions for the visualisation.
        # Snake the iteration so the resulting curve is continuous (left-to-
        # right on even rows, right-to-left on odd rows).
        ee_path: list[np.ndarray]      = []
        pen_tip_path: list[np.ndarray] = []
        target_path: list[np.ndarray]  = []  # commanded EE positions

        for iy, py in enumerate(coords):
            ix_order = range(n) if iy % 2 == 0 else range(n - 1, -1, -1)
            for ix in ix_order:
                px = coords[ix]
                # On paper (paper_z = 0).
                offset = paper_R @ np.array(
                    [px / 1000.0, py / 1000.0, 0.0]
                )
                T_des = np.eye(4)
                T_des[:3, :3] = R_begin
                T_des[:3,  3] = T_begin[:3, 3] + offset
                q_sol, resid, conv = solve_ik(
                    chain, T_des, q_begin,
                    use_null_space=True, params=ik_params,
                )
                results_paper[iy, ix] = conv
                resids_paper[iy, ix]  = resid

                if conv:
                    _, T_sol = chain.fk(q_sol)
                    R_sol = T_sol[:3, :3]
                    ee_pos = T_sol[:3, 3]
                    pen_tip = (ee_pos
                               + self.pen_offset_m
                               * (R_sol @ self.pen_axis_local))
                    ee_path.append(ee_pos)
                    pen_tip_path.append(pen_tip)
                    target_path.append(T_des[:3, 3])

                # Lifted (paper_z = +lift_mm so EE moves up away from
                # the horizontal table).
                offset = paper_R @ np.array(
                    [px / 1000.0, py / 1000.0, self.lift_mm / 1000.0]
                )
                T_des = np.eye(4)
                T_des[:3, :3] = R_begin
                T_des[:3,  3] = T_begin[:3, 3] + offset
                _, _, conv_lift = solve_ik(
                    chain, T_des, q_begin,
                    use_null_space=True, params=ik_params,
                )
                results_lift[iy, ix] = conv_lift

        self._print_map(
            'REACHABILITY MAP — paper plane (pz=0)', coords, results_paper
        )
        self._print_map(
            f'REACHABILITY MAP — lift plane (pz=+{self.lift_mm:.0f} mm)',
            coords, results_lift
        )

        # Largest axis-aligned square centred on (0,0) that's fully reachable
        # on both planes — that's a safe workspace_x_mm = workspace_y_mm pick.
        both = results_paper & results_lift
        max_half = 0.0
        center_ix = n // 2
        for half_steps in range(1, center_ix + 1):
            lo, hi = center_ix - half_steps, center_ix + half_steps
            if both[lo:hi+1, lo:hi+1].all():
                max_half = half_steps * self.step
            else:
                break

        # Largest axis-aligned X / Y extent that contains the centre on
        # both planes (gives an asymmetric box if reachability isn't square).
        def _extent_along(axis_results):
            # axis_results is a 1-D row through the centre.
            cx = len(axis_results) // 2
            lo = cx
            while lo > 0 and axis_results[lo - 1]:
                lo -= 1
            hi = cx
            while hi < len(axis_results) - 1 and axis_results[hi + 1]:
                hi += 1
            return coords[lo], coords[hi]

        if both[center_ix, center_ix]:
            x_lo, x_hi = _extent_along(both[center_ix, :])
            y_lo, y_hi = _extent_along(both[:, center_ix])
            self.get_logger().info(
                f'Safe centred square: '
                f'workspace_x_mm = workspace_y_mm = {2 * max_half:.0f} mm'
            )
            self.get_logger().info(
                f'Axis-aligned extents through centre: '
                f'X=[{x_lo:+.0f}, {x_hi:+.0f}] mm, '
                f'Y=[{y_lo:+.0f}, {y_hi:+.0f}] mm'
            )
        else:
            self.get_logger().error(
                'Canvas centre itself is NOT reachable — begin_draw_joints '
                'are not a valid IK solution under the current weights. '
                'Try lowering joint_weights or moving begin_draw_joints '
                'away from joint limits.'
            )

        # Total reachable count for the "is the workspace mostly empty?" feel.
        total = n * n
        ok_paper = int(results_paper.sum())
        ok_both  = int(both.sum())
        self.get_logger().info(
            f'Coverage: {ok_paper}/{total} on paper, '
            f'{ok_both}/{total} on both paper + lift'
        )

        # 3D plot for eyeball debugging — robot pose + EE/pen tip paths.
        self._visualize(chain, q_begin,
                        np.array(ee_path) if ee_path else None,
                        np.array(pen_tip_path) if pen_tip_path else None,
                        np.array(target_path) if target_path else None)

    def _print_map(self, label: str, coords: np.ndarray,
                   results: np.ndarray) -> None:
        n = len(coords)
        self.get_logger().info('')
        self.get_logger().info(label)
        # Column header: X coords every 4th cell.
        header = '     '
        for ix in range(n):
            header += f'{int(coords[ix]):+4d}' if ix % 4 == 0 else '    '
        self.get_logger().info(header + '  (mm, paper X)')
        # Map rows: high Y at top so it reads like a screen.
        for iy in range(n - 1, -1, -1):
            row = f'{int(coords[iy]):+4d} '
            for ix in range(n):
                row += '  . ' if results[iy, ix] else '  X '
            self.get_logger().info(row)

    def _visualize(self, chain: UrdfChain, q_begin: np.ndarray,
                   ee_path, pen_tip_path, target_path) -> None:
        """Open a matplotlib window with 4 panes:
          - Top-left: 3D view of robot + paths (orthographic projection).
          - Top-right: top-down XY view (what the drawing looks like on the
            table).
          - Bottom-left: front XZ view (forward reach + height).
          - Bottom-right: side YZ view (lateral position + height).
        2D projections are 1:1 aspect and let you actually read off
        positions without depth-flattening confusion.
        """
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        except Exception as e:
            self.get_logger().warn(
                f'Could not import matplotlib ({type(e).__name__}: {e}). '
                f'Run `python3 -c "import matplotlib; print(matplotlib.__version__)"` '
                f'to see the actual error, or upgrade with '
                f'`pip3 install --user --upgrade matplotlib`.'
            )
            return

        # ── Compute robot geometry + paths once ──────────────────────────
        T_at_origin, T_ee = chain.fk(q_begin)
        link_pts = np.array([T[:3, 3] for T in T_at_origin]
                            + [T_ee[:3, 3]])
        ee0 = T_ee[:3, 3]
        pen0 = ee0 + self.pen_offset_m * (T_ee[:3, :3] @ self.pen_axis_local)

        # Bounding box for setting axis limits consistently across panes.
        all_pts = link_pts.copy()
        if ee_path is not None and len(ee_path) > 0:
            all_pts = np.vstack([all_pts, ee_path])
        if pen_tip_path is not None and len(pen_tip_path) > 0:
            all_pts = np.vstack([all_pts, pen_tip_path])
        mn, mx = all_pts.min(axis=0), all_pts.max(axis=0)
        ctr = (mn + mx) / 2.0
        half = max((mx - mn).max() / 2.0, 0.15) + 0.05

        # ── Figure layout: 2 × 2 grid ────────────────────────────────────
        fig = plt.figure(figsize=(14, 11))
        ax3d  = fig.add_subplot(2, 2, 1, projection='3d')
        ax_xy = fig.add_subplot(2, 2, 2)
        ax_xz = fig.add_subplot(2, 2, 3)
        ax_yz = fig.add_subplot(2, 2, 4)

        # ── 3D pane ──────────────────────────────────────────────────────
        ax3d.set_proj_type('ortho')   # orthographic kills perspective confusion
        ax3d.view_init(elev=25, azim=-60)
        ax3d.plot(link_pts[:, 0], link_pts[:, 1], link_pts[:, 2],
                  color='black', linewidth=2.5, zorder=5, label='Links')
        for i, T in enumerate(T_at_origin):
            ji = chain.joints[i]
            if ji['type'] not in ('revolute', 'continuous'):
                continue
            axis_base = T[:3, :3] @ ji['axis']
            color = 'tab:orange' if ji['type'] == 'revolute' else 'tab:cyan'
            self._draw_cylinder(ax3d, T[:3, 3], axis_base,
                                radius=0.018, length=0.05, color=color)
        ax3d.scatter([ee0[0]], [ee0[1]], [ee0[2]], color='black', s=120,
                     marker='*', zorder=6, label='EE @ begin_draw')
        ax3d.scatter([pen0[0]], [pen0[1]], [pen0[2]], color='blue', s=120,
                     marker='*', zorder=6, label='Pen tip @ begin_draw')
        ax3d.plot([ee0[0], pen0[0]], [ee0[1], pen0[1]], [ee0[2], pen0[2]],
                  color='blue', linewidth=1.5, alpha=0.5)
        if ee_path is not None and len(ee_path) > 1:
            ax3d.plot(ee_path[:, 0], ee_path[:, 1], ee_path[:, 2],
                      color='red', linewidth=1, alpha=0.8,
                      label=f'EE path ({len(ee_path)} pts)')
        if pen_tip_path is not None and len(pen_tip_path) > 1:
            ax3d.plot(pen_tip_path[:, 0], pen_tip_path[:, 1],
                      pen_tip_path[:, 2], color='blue', linewidth=1,
                      alpha=0.8, label='Pen tip path')
        # Floor outline.
        f = 0.5
        floor = np.array([[-f, -f, 0], [+f, -f, 0], [+f, +f, 0],
                          [-f, +f, 0], [-f, -f, 0]])
        ax3d.plot(floor[:, 0], floor[:, 1], floor[:, 2],
                  color='green', linestyle='--', alpha=0.4, label='Floor')
        ax3d.set_xlabel('X (m)')
        ax3d.set_ylabel('Y (m)')
        ax3d.set_zlabel('Z (m)')
        ax3d.set_xlim(ctr[0] - half, ctr[0] + half)
        ax3d.set_ylim(ctr[1] - half, ctr[1] + half)
        ax3d.set_zlim(max(0.0, ctr[2] - half), ctr[2] + half)
        ax3d.set_title('3D — drag to rotate')
        ax3d.legend(loc='upper right', fontsize=7)

        # ── 2D projection helper ─────────────────────────────────────────
        def _plot_2d(ax, ai: int, bi: int, alabel: str, blabel: str,
                     title: str, floor_axis: int):
            # Links
            ax.plot(link_pts[:, ai], link_pts[:, bi],
                    color='black', linewidth=2.0, zorder=3)
            ax.scatter(link_pts[:-1, ai], link_pts[:-1, bi],
                       color='gray', s=40, zorder=4)
            # EE + pen tip
            ax.scatter([ee0[ai]], [ee0[bi]], color='black', s=100,
                       marker='*', zorder=5)
            ax.scatter([pen0[ai]], [pen0[bi]], color='blue', s=100,
                       marker='*', zorder=5)
            ax.plot([ee0[ai], pen0[ai]], [ee0[bi], pen0[bi]],
                    color='blue', linewidth=1.5, alpha=0.5)
            # Paths
            if ee_path is not None and len(ee_path) > 0:
                ax.plot(ee_path[:, ai], ee_path[:, bi],
                        color='red', linewidth=1, alpha=0.7)
                ax.scatter(ee_path[:, ai], ee_path[:, bi],
                           color='red', s=4, alpha=0.5)
            if pen_tip_path is not None and len(pen_tip_path) > 0:
                ax.plot(pen_tip_path[:, ai], pen_tip_path[:, bi],
                        color='blue', linewidth=1, alpha=0.7)
                ax.scatter(pen_tip_path[:, ai], pen_tip_path[:, bi],
                           color='blue', s=4, alpha=0.5)
            # Floor line (only when one axis is base Z).
            if floor_axis == ai:
                ax.axvline(x=0, color='green', linestyle='--', alpha=0.4)
            elif floor_axis == bi:
                ax.axhline(y=0, color='green', linestyle='--', alpha=0.4)
            ax.set_xlabel(alabel + ' (m)')
            ax.set_ylabel(blabel + ' (m)')
            ax.set_title(title)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            # Same bounding box as the 3D view.
            ax.set_xlim(ctr[ai] - half, ctr[ai] + half)
            ax.set_ylim(ctr[bi] - half, ctr[bi] + half)

        _plot_2d(ax_xy, 0, 1, 'X', 'Y', 'Top-down  XY  (table surface)',
                 floor_axis=2)   # Z is "out of page" — no floor line here.
        _plot_2d(ax_xz, 0, 2, 'X', 'Z', 'Front  XZ  (height vs forward)',
                 floor_axis=2)
        _plot_2d(ax_yz, 1, 2, 'Y', 'Z', 'Side  YZ  (height vs lateral)',
                 floor_axis=2)

        fig.suptitle('Robot at begin_draw + IK-probed paths '
                     '(red=EE, blue=pen tip, green=floor)', fontsize=11)
        fig.tight_layout()
        self.get_logger().info('Opening matplotlib window — close it to exit.')
        plt.show()

    @staticmethod
    def _draw_cylinder(ax, center, axis, radius=0.015, length=0.04,
                       color='blue') -> None:
        """Draw a short solid cylinder centred on `center`, oriented along
        `axis` (a 3-vector in base frame). Used to mark joint rotation axes.
        """
        axis = np.asarray(axis, dtype=float)
        axis = axis / np.linalg.norm(axis)
        # Pick any vector not collinear with axis, then orthogonalise.
        ref = (np.array([0.0, 0.0, 1.0]) if abs(axis[2]) < 0.9
               else np.array([1.0, 0.0, 0.0]))
        bx = np.cross(axis, ref)
        bx /= np.linalg.norm(bx)
        by = np.cross(axis, bx)

        n_theta = 16
        theta = np.linspace(0.0, 2.0 * np.pi, n_theta)
        h = np.linspace(-length / 2.0, length / 2.0, 2)
        TT, HH = np.meshgrid(theta, h)
        X = (center[0]
             + radius * (np.cos(TT) * bx[0] + np.sin(TT) * by[0])
             + HH * axis[0])
        Y = (center[1]
             + radius * (np.cos(TT) * bx[1] + np.sin(TT) * by[1])
             + HH * axis[1])
        Z = (center[2]
             + radius * (np.cos(TT) * bx[2] + np.sin(TT) * by[2])
             + HH * axis[2])
        ax.plot_surface(X, Y, Z, color=color, alpha=0.75, edgecolor='none')


def main():
    rclpy.init()
    node = WorkspaceProber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
