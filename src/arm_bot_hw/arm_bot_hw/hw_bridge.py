#!/usr/bin/env python3
"""
hw_bridge.py — closed-loop ROS 2 ↔ DaMiao-CAN hardware bridge for the 7-DOF arm.

Owns the U2CANFD adapter (the SAME Motor_Control driver pos_motor_sub.py uses)
and bridges the teach-pendant control stack to the real motors:

  • publishes /joint_states from the LIVE motor encoders, so the IK/FK nodes and
    the pendant start from — and track — the ACTUAL arm pose;
  • drives the motors (POS_VEL) from /arm_controller/joint_trajectory — the topic
    every pendant motion command lands on (joint jog, set, move-to-target,
    drawing, and the Cartesian-jog IK via ik_to_trajectory).

SAFE START: on boot it reads the encoders and HOLDS the current pose; the arm
only moves on a deliberate command. Velocity is capped (`max_vel`). It disables
the motors on shutdown and on /hw_estop.

Reuses arm_bot_hw/lib/DM_motor.py; pos_motor_sub.py is left untouched. Requires
the U2CANFD adapter + 7 DM motors on CAN ids 0x01..0x07 (joint_1..joint_7).
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory

from .lib.DM_motor import (
    DM_Motor_Type,
    Control_Mode,
    Control_Mode_Code,
    DmActData,
    Motor_Control,
)

JOINT_NAMES = [f"joint_{i}" for i in range(1, 8)]
CAN_IDS = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
MST_IDS = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]
# Motors mounted reversed vs the URDF joint sense (same as pos_motor_sub.py).
INVERTED = {0x03, 0x05}
# joint_1..4 = DM4340, joint_5..7 = DM4310 (same as pos_motor_sub.py).
MOTOR_TYPES = [DM_Motor_Type.DM4340_48V] * 4 + [DM_Motor_Type.DM4310_48V] * 3
DEFAULT_SN = "E067CA134F67746CCA5451F1BE23BAD8"


class HwBridge(Node):
    def __init__(self):
        super().__init__("arm_hw_bridge")
        self.sn = str(self.declare_parameter("sn", DEFAULT_SN).value)
        self.max_vel = float(self.declare_parameter("max_vel", 1.5).value)
        self.rate = float(self.declare_parameter("publish_rate", 50.0).value)
        # 'encoder' = /joint_states from live encoders (closed-loop, default);
        # 'command' = echo the last command (open-loop fallback if reads are flaky).
        self.feedback = str(self.declare_parameter("feedback", "encoder").value)
        nom = int(self.declare_parameter("nom_baud", 1000000).value)
        dat = int(self.declare_parameter("dat_baud", 1000000).value)

        init = [DmActData(motorType=MOTOR_TYPES[i], mode=Control_Mode.POS_VEL_MODE,
                          can_id=CAN_IDS[i], mst_id=MST_IDS[i]) for i in range(7)]
        self.get_logger().info(f"opening U2CANFD adapter sn={self.sn} …")
        self.ctrl = Motor_Control(nom, dat, self.sn, init)   # opens adapter + enables motors
        self.cid = dict(zip(JOINT_NAMES, CAN_IDS))
        for cid in CAN_IDS:
            m = self.ctrl.getMotor(cid)
            if m is not None:
                self.ctrl.switchControlMode(m, Control_Mode_Code.POS_VEL)

        self._estopped = False
        self._traj = None       # active multi-point playback: (t0, [(t_sec, cmd), …])

        # ── SAFE START: read the actual pose and hold it (no auto-motion) ──────
        self._cmd = self._read_positions(settle=0.25)
        self.get_logger().warn(
            "SAFE START — holding current pose:  "
            + "  ".join(f"{n}={self._cmd[n]:+.3f}" for n in JOINT_NAMES)
            + "   << verify this matches the physical arm before jogging; "
              "if it reads ~0/garbage the encoder read failed — E-stop and check.")
        self._command(self._cmd)

        self.pub = self.create_publisher(JointState, "/joint_states", 20)
        self.create_subscription(JointTrajectory, "/arm_controller/joint_trajectory",
                                 self._on_traj, 10)
        self.create_subscription(Bool, "/hw_estop", self._on_estop, 10)
        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            f"arm_hw_bridge ready — driving real motors (max_vel={self.max_vel} rad/s, "
            f"feedback={self.feedback}).")

    # ── motor helpers ─────────────────────────────────────────────────────────
    def _read_positions(self, settle: float = 0.0) -> dict:
        """Request + read each encoder, returned in URDF joint-space (un-inverted)."""
        for cid in CAN_IDS:
            m = self.ctrl.getMotor(cid)
            if m is not None:
                self.ctrl.refresh_motor_status(m)
        if settle:
            time.sleep(settle)   # one-time at startup ONLY — never call with settle in the timer
        out = {}
        for i, n in enumerate(JOINT_NAMES):
            m = self.ctrl.getMotor(CAN_IDS[i])
            q = float(m.Get_Position()) if m is not None else 0.0
            out[n] = -q if CAN_IDS[i] in INVERTED else q
        return out

    def _command(self, cmd: dict, vel: float = None):
        """Command every motor toward cmd (URDF joint-space) at a capped velocity."""
        if self._estopped:
            return
        v = self.max_vel if vel is None else min(abs(vel), self.max_vel)
        for n in JOINT_NAMES:
            cid = self.cid[n]
            m = self.ctrl.getMotor(cid)
            if m is None:
                continue
            target = -cmd[n] if cid in INVERTED else cmd[n]
            self.ctrl.control_pos_vel(m, target, v)

    # ── ROS callbacks ─────────────────────────────────────────────────────────
    def _pt_to_cmd(self, pt, idx) -> dict:
        return {n: (float(pt.positions[idx[n]])
                    if n in idx and idx[n] < len(pt.positions) else self._cmd[n])
                for n in JOINT_NAMES}

    def _on_traj(self, msg: JointTrajectory):
        if not msg.points:
            return
        idx = {name: i for i, name in enumerate(msg.joint_names)}
        if len(msg.points) == 1:
            # jog / set / move / freeze — single target; POS_VEL drives to it.
            self._traj = None
            self._cmd = self._pt_to_cmd(msg.points[0], idx)
        else:
            # multi-point (e.g. drawing) — play the waypoints by time_from_start.
            pts = [(p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
                    self._pt_to_cmd(p, idx)) for p in msg.points]
            self._traj = (time.monotonic(), pts)

    def _on_estop(self, msg: Bool):
        if msg.data and not self._estopped:
            self._estopped = True
            self._traj = None
            self.ctrl.disable_all()
            self.get_logger().error("/hw_estop — motors DISABLED")
        elif not msg.data and self._estopped:
            self._estopped = False
            for cid in CAN_IDS:
                m = self.ctrl.getMotor(cid)
                if m is not None:
                    self.ctrl.switchControlMode(m, Control_Mode_Code.POS_VEL)
            self._cmd = self._read_positions(settle=0.25)
            self._command(self._cmd)
            self.get_logger().warn("/hw_estop cleared — motors re-enabled, holding pose")

    def _tick(self):
        if self._estopped:
            return
        # advance multi-point playback → _cmd
        if self._traj is not None:
            t0, pts = self._traj
            el = time.monotonic() - t0
            cur = pts[0][1]
            for t, c in pts:
                if el >= t:
                    cur = c
                else:
                    break
            self._cmd = cur
            if el >= pts[-1][0]:
                self._traj = None
        # re-send the command every tick (holds + keeps feedback frames flowing)
        self._command(self._cmd)
        # publish /joint_states
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(JOINT_NAMES)
        if self.feedback == "encoder":
            pos = self._read_positions()        # reads the latest reply (no blocking)
            js.position = [pos[n] for n in JOINT_NAMES]
        else:
            js.position = [self._cmd[n] for n in JOINT_NAMES]
        self.pub.publish(js)

    def destroy_node(self):
        try:
            self.get_logger().info("shutting down — disabling motors")
            self.ctrl.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HwBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
