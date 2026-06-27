#!/usr/bin/env python3
"""
record_encoder.py — drive the arm AND log its ENCODER feedback to a CSV, with
an automatic recording window bracketed by the begin_draw pose.

Why a subclass: while drawing, `pos_motor_sub` owns the single USB-CAN adapter,
so no separate process can read the motors. This node *is* `pos_motor_sub`
(it subclasses it, so the draw behaves identically) and additionally logs the
measured encoder position/velocity of all seven joints.

AUTO WINDOW (default on): logging starts when /joint_states settles at the
begin_draw pose and stops when it settles back at begin_draw AFTER drawing — so
the CSV is exactly begin_draw → draw → begin_draw, no go-to-start / home travel.
(The drawing planner's return_to_begin_seconds makes the arm come back to
begin_draw at the end; keep it > 0 for the stop to trigger.) The motors keep
being driven the whole time — only the CSV logging window is gated. Ctrl-C
always saves whatever has been logged. Set auto_window:=false to log everything.

    source ~/7dof_ws/install/setup.bash
    ros2 run arm_bot_hw record_encoder            # run INSTEAD of pos_motor_sub
    #   ... make it draw; logging auto-starts/stops at begin_draw ...
    #   Ctrl-C to exit (also saves if the window never closed)

Output (auto-suffixed " (2)", " (3)"):  recordings/draw_capture.csv
    columns: t_wall_s, t_rel_s,
             joint_1_cmd_rad, joint_1_enc_rad, joint_1_enc_vel_radps,  ... joint_7_*

Key params (--ros-args -p name:=value):
    begin_draw_joints (7 floats)   default [0,-0.7,0,1.4,0.01,0,1.0]  (match the planner)
    auto_window (bool)             default true
    begin_tol_rad                  default 0.12   pose-match tolerance (per joint)
    stop_confirm_s                 default 3.0    settle-at-begin hold to STOP (> stroke dwell)
"""
import csv
import os
import time

import numpy as np
import rclpy

from arm_bot_hw.pos_motor_sub import PosMotorSub

JOINT_ORDER = [f'joint_{i}' for i in range(1, 8)]


class EncoderRecorder(PosMotorSub):
    def __init__(self):
        super().__init__()                       # full motor bring-up, exactly like pos_motor_sub

        # ── auto-window params ──────────────────────────────────────────────
        self.declare_parameter('begin_draw_joints',
                               [0.0, -0.7, 0.0, 1.4, 0.01, 0.0, 1.0])
        self.declare_parameter('auto_window', True)
        self.declare_parameter('begin_tol_rad', 0.20)   # START match tol vs begin_draw param
                                                        # (generous — the held pose is the
                                                        #  pen-up HOVER, raised from the param;
                                                        #  still far from home/go-to-start)
        self.declare_parameter('return_tol_rad', 0.05)  # STOP match tol vs the CAPTURED begin hold
        self.declare_parameter('still_eps_rad', 0.002)  # per-tick move below = "still"
        self.declare_parameter('start_confirm_s', 0.5)  # settle-at-begin to START
        self.declare_parameter('min_draw_s', 1.0)       # motion after start before a STOP is allowed
        self.declare_parameter('stop_confirm_s', 3.0)   # settle-at-begin to STOP (> stroke dwell)
        gp = lambda n: self.get_parameter(n).value      # noqa: E731
        self.q_begin = np.array(gp('begin_draw_joints'), dtype=float)
        self.auto_window = bool(gp('auto_window'))
        self.begin_tol = float(gp('begin_tol_rad'))
        self.return_tol = float(gp('return_tol_rad'))
        self.still_eps = float(gp('still_eps_rad'))
        self.start_confirm_s = float(gp('start_confirm_s'))
        self.min_draw_s = float(gp('min_draw_s'))
        self.stop_confirm_s = float(gp('stop_confirm_s'))

        # ── window state machine ────────────────────────────────────────────
        # 'armed' = waiting to reach begin_draw; 'recording'; 'done'.
        self._state = 'armed' if self.auto_window else 'recording'
        self._q_prev = None
        self._q_start = None      # the exact begin_draw hold pose captured at START
        self._t_prev = None
        self._still_t = 0.0       # cumulative seconds the command has held still
        self._drew_t = 0.0        # cumulative seconds of motion since recording began

        # ── CSV ─────────────────────────────────────────────────────────────
        outdir = os.environ.get('ENCODER_LOG_DIR', os.path.expanduser('~/7dof_ws/recordings'))
        os.makedirs(outdir, exist_ok=True)
        name = os.environ.get('ENCODER_LOG_NAME', 'draw_capture')
        self.path = self._unique_path(outdir, name)
        self._f = open(self.path, 'w', newline='')
        self._w = csv.writer(self._f)
        header = ['t_wall_s', 't_rel_s']
        for j in JOINT_ORDER:
            header += [f'{j}_cmd_rad', f'{j}_enc_rad', f'{j}_enc_vel_radps']
        self._w.writerow(header)
        self._closed = False
        self._t0 = None
        self._rows = 0

        if self.auto_window:
            self.get_logger().info(
                f'ENCODER RECORDER armed — waiting for begin_draw pose '
                f'(tol {self.begin_tol:.2f} rad) -> {self.path}')
        else:
            self.get_logger().info(f'ENCODER RECORDING (whole session) -> {self.path}')

    # ── helpers ─────────────────────────────────────────────────────────────
    @staticmethod
    def _unique_path(outdir, name):
        p = os.path.join(outdir, f'{name}.csv')
        if not os.path.exists(p):
            return p
        i = 2
        while os.path.exists(os.path.join(outdir, f'{name} ({i}).csv')):
            i += 1
        return os.path.join(outdir, f'{name} ({i}).csv')

    def _cmd_vector(self, msg):
        """Commanded joint positions in canonical order, or None if incomplete."""
        idx = {n: i for i, n in enumerate(msg.name)}
        try:
            return np.array([msg.position[idx[j]] for j in JOINT_ORDER], dtype=float)
        except (KeyError, IndexError):
            return None

    def _write_row(self, now, q_cmd):
        if self._t0 is None:
            self._t0 = now
        row = [f'{now:.6f}', f'{now - self._t0:.6f}']
        for k, j in enumerate(JOINT_ORDER):
            can_id = self.joint_canid_map.get(j)
            motor = self.control.getMotor(can_id) if can_id is not None else None
            enc = vel = float('nan')
            if motor is not None:
                enc = motor.Get_Position()
                vel = motor.Get_Velocity()
                if can_id in self.inverted_motors:    # undo command-side inversion
                    enc = -enc
                    vel = -vel
            row += [f'{q_cmd[k]:.6f}', f'{enc:.6f}', f'{vel:.6f}']
        self._w.writerow(row)
        self._rows += 1
        if self._rows % 200 == 0:
            self._f.flush()

    def _save(self, reason):
        if self._closed:
            return
        self._closed = True
        try:
            self._f.flush()
            self._f.close()
        except Exception:                             # noqa: BLE001
            pass
        self.get_logger().info(f'window {reason}: saved {self._rows} encoder samples -> {self.path}')

    # ── main callback ───────────────────────────────────────────────────────
    def joint_states_callback(self, msg):
        # 1) drive the motors exactly as the parent does (and refresh encoders).
        super().joint_states_callback(msg)
        if self._state == 'done':
            return

        q_cmd = self._cmd_vector(msg)
        if q_cmd is None or q_cmd.shape[0] != self.q_begin.shape[0]:
            return

        now = time.time()
        dt = 0.0 if self._t_prev is None else max(0.0, now - self._t_prev)
        self._t_prev = now

        # "still" = the commanded pose barely moved since last tick.
        moved_step = (np.inf if self._q_prev is None
                      else float(np.max(np.abs(q_cmd - self._q_prev))))
        self._q_prev = q_cmd
        still = moved_step < self.still_eps
        self._still_t = self._still_t + dt if still else 0.0
        near_begin = float(np.max(np.abs(q_cmd - self.q_begin))) < self.begin_tol

        if self._state == 'armed':
            # START once settled at the begin_draw pose; capture the exact held
            # pose so the STOP can match the return tightly.
            if near_begin and self._still_t >= self.start_confirm_s:
                self._state = 'recording'
                self._q_start = q_cmd.copy()
                self._t0 = None
                self._drew_t = 0.0
                self.get_logger().info('window START — at begin_draw, logging…')
                self._write_row(now, q_cmd)
            return

        # state == 'recording'
        if not still:
            self._drew_t += dt
        self._write_row(now, q_cmd)
        # STOP once the arm has actually drawn, then settled back at the captured
        # begin_draw hold pose.
        near_start = float(np.max(np.abs(q_cmd - self._q_start))) < self.return_tol
        if (self._drew_t >= self.min_draw_s and near_start
                and self._still_t >= self.stop_confirm_s):
            self._state = 'done'
            self._save('STOP (returned to begin_draw)')

    def destroy_node(self):
        self._save('Ctrl-C')                          # no-op if already saved
        super().destroy_node()                        # disables the motors


def main(args=None):
    rclpy.init(args=args)
    node = EncoderRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl-C received — saving encoder log.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
