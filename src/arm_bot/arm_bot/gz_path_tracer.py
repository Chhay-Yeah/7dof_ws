#!/usr/bin/env python3
"""
gz_path_tracer.py — render EE and pen-tip trails inside Gazebo by spawning
small coloured sphere entities ("breadcrumbs") as the robot moves.

Why not the /marker service?
  Ignition Fortress's GUI MarkerManager often registers /marker but fails
  to respond — the service hangs forever. Spawning real entities via
  /world/<world>/create works reliably because it goes to the server
  process directly, not the GUI's render thread.

Trade-offs:
  - Each breadcrumb is a separate SDF entity. They accumulate, so we cap
    the count per channel (default 200) and remove the oldest when over.
  - Subprocess overhead (~50–100 ms per `ign service` call). To keep that
    off the ROS callback thread, spawns/removes run in daemon threads.
  - Breadcrumbs persist after this node dies. Restart Gazebo to clear.

Subscribes:
  /robot_description (latched URDF)
  /joint_states
"""
import shutil
import subprocess
import threading
from collections import deque
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from arm_bot.ik_lib import UrdfChain


def _make_sphere_sdf(name: str, radius_m: float, rgb: tuple) -> str:
    """Inline SDF for a tiny static visual sphere — no collision, no inertia."""
    return (
        '<?xml version="1.0"?>'
        '<sdf version="1.6">'
        f'<model name="{name}">'
        '<static>true</static>'
        '<link name="link">'
        '<visual name="visual">'
        f'<geometry><sphere><radius>{radius_m:.4f}</radius></sphere></geometry>'
        '<material>'
        f'<ambient>{rgb[0]:.3f} {rgb[1]:.3f} {rgb[2]:.3f} 1</ambient>'
        f'<diffuse>{rgb[0]:.3f} {rgb[1]:.3f} {rgb[2]:.3f} 1</diffuse>'
        '</material>'
        '</visual>'
        '</link>'
        '</model>'
        '</sdf>'
    )


def _escape_protobuf_string(s: str) -> str:
    """Backslash-escape a string so it can be embedded in a protobuf
    text-format quoted field (used for ign service -r '...')."""
    return s.replace('\\', '\\\\').replace('"', '\\"')


class GzPathTracer(Node):
    def __init__(self):
        super().__init__('gz_path_tracer')

        self.declare_parameter('base_link',         'base_link')
        self.declare_parameter('tip_link',          'ee')
        self.declare_parameter('pen_axis_local',    [0.0, 0.0, 1.0])
        self.declare_parameter('pen_offset_mm',     100.0)
        # World name (matches `gz_args` in gazebo.launch.py; default world
        # is "empty" because we launch with empty.sdf).
        self.declare_parameter('world_name',        'empty')
        # Spawn a breadcrumb whenever EE or pen tip moves at least this far
        # from the last breadcrumb. Filters out stationary jitter.
        self.declare_parameter('min_step_mm',       5.0)
        # Per-channel cap. Each crumb adds ~1 ms physics-update cost; 200
        # is safe. Trail length = max_crumbs * min_step_mm in mm.
        self.declare_parameter('max_crumbs',        1)
        self.declare_parameter('crumb_radius_m',    0.003)
        self.declare_parameter('ee_rgb',            [1.0, 0.15, 0.15])
        self.declare_parameter('pen_rgb',           [0.15, 0.4, 1.0])
        # 'ign' for Fortress, 'gz' for Garden / Harmonic.
        self.declare_parameter('gz_cli',            'ign')
        # FK / motion-check tick period.
        self.declare_parameter('check_period_s',    1.0)

        gp = lambda n: self.get_parameter(n).value
        self.base_link      = gp('base_link')
        self.tip_link       = gp('tip_link')
        self.pen_axis_local = np.array(gp('pen_axis_local'), dtype=float)
        self.pen_axis_local /= np.linalg.norm(self.pen_axis_local)
        self.pen_offset_m   = float(gp('pen_offset_mm')) / 1000.0
        self.world_name     = gp('world_name')
        self.min_step_m     = float(gp('min_step_mm')) / 1000.0
        self.max_crumbs     = int(gp('max_crumbs'))
        self.crumb_radius_m = float(gp('crumb_radius_m'))
        self.ee_rgb         = tuple(gp('ee_rgb'))
        self.pen_rgb        = tuple(gp('pen_rgb'))
        self.gz_cli         = gp('gz_cli')
        self.check_period_s = float(gp('check_period_s'))

        self.create_endpoint = f'/world/{self.world_name}/create'
        self.remove_endpoint = f'/world/{self.world_name}/remove'

        if not shutil.which(self.gz_cli):
            self.get_logger().error(
                f"`{self.gz_cli}` CLI not found in PATH — can't spawn "
                f"breadcrumbs. The node will run but produce nothing in Gazebo."
            )

        self._chain: Optional[UrdfChain] = None
        self._joint_index = None
        self._latest_q: Optional[np.ndarray] = None
        self._lock = threading.Lock()

        # Per-channel: deque of spawned entity names + last-spawned position.
        self._ee_crumbs:  deque = deque()
        self._pen_crumbs: deque = deque()
        self._last_ee:  Optional[np.ndarray] = None
        self._last_pen: Optional[np.ndarray] = None
        self._next_id = 0
        self._id_lock = threading.Lock()
        # Print the first spawn's request + result so we can see exactly
        # what gets sent to ign and what comes back. Set to False after
        # the first successful (or failed) call.
        self._debug_first = True
        self._debug_lock = threading.Lock()

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(String, '/robot_description',
                                 self._cb_urdf, latched)
        self.create_subscription(JointState, '/joint_states',
                                 self._cb_joints, 30)
        self.create_timer(self.check_period_s, self._tick)

        self.get_logger().info(
            f'gz_path_tracer ready — dropping breadcrumbs every '
            f'{gp("min_step_mm"):.1f} mm of motion, capped at '
            f'{self.max_crumbs} per channel.'
        )

    # ── Subscriptions ──────────────────────────────────────────────────

    def _cb_urdf(self, msg: String) -> None:
        if self._chain is not None:
            return
        try:
            chain = UrdfChain(msg.data, self.base_link, self.tip_link)
        except Exception as e:
            self.get_logger().error(f'URDF parse failed: {e}')
            return
        self._chain = chain
        self.get_logger().info(f'URDF loaded: {chain.n} DoF')

    def _cb_joints(self, msg: JointState) -> None:
        if self._chain is None:
            return
        if self._joint_index is None:
            try:
                self._joint_index = [msg.name.index(n)
                                     for n in self._chain.joint_names]
            except ValueError:
                return
        q = np.array([msg.position[i] for i in self._joint_index], dtype=float)
        with self._lock:
            self._latest_q = q

    # ── Motion-tick: drop breadcrumbs ──────────────────────────────────

    def _tick(self) -> None:
        with self._lock:
            if self._chain is None or self._latest_q is None:
                return
            q = self._latest_q.copy()
            chain = self._chain
        _, T_ee = chain.fk(q)
        ee  = T_ee[:3, 3]
        pen = ee + self.pen_offset_m * (T_ee[:3, :3] @ self.pen_axis_local)

        if (self._last_ee is None
                or np.linalg.norm(ee - self._last_ee) >= self.min_step_m):
            self._drop_async(ee,  self.ee_rgb,  self._ee_crumbs,  'ee')
            self._last_ee = ee
        if (self._last_pen is None
                or np.linalg.norm(pen - self._last_pen) >= self.min_step_m):
            self._drop_async(pen, self.pen_rgb, self._pen_crumbs, 'pen')
            self._last_pen = pen

    # ── Breadcrumb spawn / remove (async, daemon threads) ──────────────

    def _next_name(self, channel: str) -> str:
        with self._id_lock:
            n = self._next_id
            self._next_id += 1
        return f'crumb_{channel}_{n}'

    def _drop_async(self, pos: np.ndarray, rgb: tuple,
                    crumbs: deque, channel: str) -> None:
        name = self._next_name(channel)
        crumbs.append(name)
        # Spawn the new crumb.
        threading.Thread(
            target=self._spawn_sync,
            args=(name, pos, rgb),
            daemon=True,
        ).start()
        # If we're over the cap, fire-and-forget remove the oldest.
        while len(crumbs) > self.max_crumbs:
            old = crumbs.popleft()
            threading.Thread(
                target=self._remove_sync,
                args=(old,),
                daemon=True,
            ).start()

    def _spawn_sync(self, name: str, pos: np.ndarray, rgb: tuple) -> None:
        sdf = _make_sphere_sdf(name, self.crumb_radius_m, rgb)
        # Match the protobuf text format used by the manual test command:
        # commas between scalar fields. Some Ignition parsers are strict.
        req = (
            f'sdf: "{_escape_protobuf_string(sdf)}", '
            f'name: "{name}", '
            f'pose: {{position: {{'
            f'x: {pos[0]:.4f}, y: {pos[1]:.4f}, z: {pos[2]:.4f}'
            f'}}}}'
        )
        self._run_service(
            self.create_endpoint,
            'ignition.msgs.EntityFactory',
            req,
        )

    def _remove_sync(self, name: str) -> None:
        req = f'name: "{name}", type: MODEL'
        self._run_service(
            self.remove_endpoint,
            'ignition.msgs.Entity',
            req,
        )

    def _run_service(self, endpoint: str, reqtype: str, req: str) -> None:
        # On the very first call, dump the full request + result so we can
        # see what's actually flowing to ign. Costs nothing after that.
        with self._debug_lock:
            debug = self._debug_first
            self._debug_first = False
        if debug:
            self.get_logger().info(
                f'First service call:\n'
                f'  endpoint = {endpoint}\n'
                f'  reqtype  = {reqtype}\n'
                f'  req      = {req[:600]}'
                + (' ... (truncated)' if len(req) > 600 else '')
            )
        try:
            proc = subprocess.run(
                [self.gz_cli, 'service',
                 '-s', endpoint,
                 '--reqtype', reqtype,
                 '--reptype', 'ignition.msgs.Boolean',
                 '--timeout', '1000',
                 '-r', req],
                check=False,
                capture_output=True,
                text=True,
                timeout=2.0,
            )
            if debug:
                self.get_logger().info(
                    f'First service result: '
                    f'returncode={proc.returncode}, '
                    f'stdout={proc.stdout.strip()[:300]!r}, '
                    f'stderr={proc.stderr.strip()[:300]!r}'
                )
            if proc.returncode != 0 and proc.stderr:
                self.get_logger().warn(
                    f'{endpoint} failed: {proc.stderr.strip()[:200]}',
                    throttle_duration_sec=5.0,
                )
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                f'{endpoint} timed out',
                throttle_duration_sec=5.0,
            )
        except FileNotFoundError:
            pass  # already warned at startup


def main():
    rclpy.init()
    node = GzPathTracer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
