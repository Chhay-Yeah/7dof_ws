#!/usr/bin/env python3
"""
record_encoder.py — drive the arm AND log its ENCODER feedback to a CSV.

Why a subclass: while drawing, `pos_motor_sub` owns the single USB-CAN adapter,
so no separate process can read the motors. This node *is* `pos_motor_sub`
(it subclasses it, so the draw behaves identically) and additionally logs the
measured encoder position/velocity of all seven joints every control tick. The
friend's `pos_motor_sub.py` is left untouched.

Run this INSTEAD of `pos_motor_sub` when you want a recording:

    source ~/7dof_ws/install/setup.bash
    ros2 run arm_bot_hw record_encoder            # then make it draw
    #   ... robot draws ...
    #   Ctrl-C when it finishes  ->  saves the CSV and prints the path

Output (auto-suffixed " (2)", " (3)" so nothing is overwritten):

    recordings/draw_capture.csv
    columns: t_wall_s, t_rel_s,
             joint_1_cmd_rad, joint_1_enc_rad, joint_1_enc_vel_radps,  ... joint_7_*

  cmd  = the commanded position from /joint_states (the trajectory being mirrored)
  enc  = the MEASURED encoder position from the motor (inversion undone)
  vel  = the MEASURED encoder velocity from the motor (real motor velocity)

Override the output dir/name with env vars ENCODER_LOG_DIR / ENCODER_LOG_NAME.
"""
import csv
import os
import time

import rclpy

from arm_bot_hw.pos_motor_sub import PosMotorSub

JOINT_ORDER = [f'joint_{i}' for i in range(1, 8)]


class EncoderRecorder(PosMotorSub):
    def __init__(self):
        super().__init__()                       # full motor bring-up, exactly like pos_motor_sub

        outdir = os.environ.get('ENCODER_LOG_DIR',
                                os.path.expanduser('~/7dof_ws/recordings'))
        os.makedirs(outdir, exist_ok=True)
        name = os.environ.get('ENCODER_LOG_NAME', 'draw_capture')
        self.path = self._unique_path(outdir, name)

        self._f = open(self.path, 'w', newline='')
        self._w = csv.writer(self._f)
        header = ['t_wall_s', 't_rel_s']
        for j in JOINT_ORDER:
            header += [f'{j}_cmd_rad', f'{j}_enc_rad', f'{j}_enc_vel_radps']
        self._w.writerow(header)

        self._t0 = None
        self._rows = 0
        self.get_logger().info(f'ENCODER RECORDING -> {self.path}  (Ctrl-C to save)')

    @staticmethod
    def _unique_path(outdir, name):
        p = os.path.join(outdir, f'{name}.csv')
        if not os.path.exists(p):
            return p
        i = 2
        while os.path.exists(os.path.join(outdir, f'{name} ({i}).csv')):
            i += 1
        return os.path.join(outdir, f'{name} ({i}).csv')

    def joint_states_callback(self, msg):
        # 1) drive the motors exactly as the parent does (this also triggers the
        #    motor responses that refresh each motor's encoder state).
        super().joint_states_callback(msg)

        # 2) log: commanded (from the incoming message) + measured (from the motor)
        now = time.time()
        if self._t0 is None:
            self._t0 = now
        cmd = {n: (msg.position[i] if i < len(msg.position) else float('nan'))
               for i, n in enumerate(msg.name)}

        row = [f'{now:.6f}', f'{now - self._t0:.6f}']
        for j in JOINT_ORDER:
            can_id = self.joint_canid_map.get(j)
            motor = self.control.getMotor(can_id) if can_id is not None else None
            enc = float('nan')
            vel = float('nan')
            if motor is not None:
                enc = motor.Get_Position()
                vel = motor.Get_Velocity()
                if can_id in self.inverted_motors:   # undo the command-side inversion
                    enc = -enc
                    vel = -vel
            row += [f'{cmd.get(j, float("nan")):.6f}', f'{enc:.6f}', f'{vel:.6f}']
        self._w.writerow(row)
        self._rows += 1
        if self._rows % 200 == 0:
            self._f.flush()

    def destroy_node(self):
        try:
            self._f.flush()
            self._f.close()
            self.get_logger().info(f'saved {self._rows} encoder samples -> {self.path}')
        except Exception:                            # noqa: BLE001
            pass
        super().destroy_node()                       # disables the motors


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
