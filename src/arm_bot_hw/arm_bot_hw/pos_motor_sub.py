import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .lib.DM_motor import (
    DM_Motor_Type,
    Control_Mode,
    Control_Mode_Code,
    DmActData,
    DM_REG,
    Motor_Control,
)


MAX_VEL = 5.0             # rad/s, hard cap on the commanded velocity
MIN_VEL = 0.05            # rad/s, small floor so a sent command never has a 0 cap
POS_FILTER_ALPHA = 1.0    # EMA weight for the target position. 1.0 = OFF (pass the
                          # commanded position straight through). A value < 1.0 low-passes
                          # the per-joint target, which LAGS each joint by an amount that
                          # grows with its speed. That is fine for holding still but breaks
                          # COORDINATED multi-joint moves: during a drawing's reach sweep
                          # joint_2 and joint_4 move at different speeds, so unequal lag
                          # desynchronises them and the pen drifts off the constant-Z paper
                          # plane (vertical lines lift; horizontal lines, which only move
                          # the gravity-free base-yaw joint, are unaffected). Hold-still
                          # jitter is handled by POS_DEADBAND_RAD + the velocity cap instead,
                          # so the position itself must NOT be filtered. Drop to ~0.85 only
                          # if rest jitter returns and the deadband isn't enough.
VEL_FILTER_ALPHA = 0.50   # EMA weight for the derived velocity cap (used only as a floor)
VEL_HEADROOM = 1.5        # the velocity LIMIT must cover the per-cycle position demand,
                          # or POS_VEL throttles and the joint lags. On a curve the joint
                          # speeds change continuously, so a lagged cap under-shoots and
                          # joints desync → the pen drifts off the paper plane (circles
                          # suffer; straight constant-speed strokes don't). Cap = max(EMA
                          # floor, instantaneous demand * this headroom).
POS_DEADBAND_RAD = 0.0015  # ~0.09 deg; skip re-commanding changes smaller than this
STALE_DT_S = 0.5          # gap above which dt is treated as a restart (reseed, no vel)

POS_VEL_ACC = 200.0
POS_VEL_DEC = 200.0


class PosMotorSub(Node):
    def __init__(self):
        super().__init__('pos_motor_sub')

        # Per-joint filter state (keyed by CAN id) for the slow-speed jitter
        # suppression in joint_states_callback.
        self._pos_f: dict[int, float] = {}      # EMA-filtered target position
        self._vel_f: dict[int, float] = {}      # EMA-filtered velocity cap
        self._last_sent: dict[int, float] = {}  # last position actually commanded
        self._last_time: float | None = None    # monotonic time of previous callback

        # ── Motor IDs ──────────────────────────────────────────────────────────
        canid1 = 0x01;  mstid1 = 0x11
        canid2 = 0x02;  mstid2 = 0x12
        canid3 = 0x03;  mstid3 = 0x13
        canid4 = 0x04;  mstid4 = 0x14
        canid5 = 0x05;  mstid5 = 0x15
        canid6 = 0x06;  mstid6 = 0x16
        canid7 = 0x07;  mstid7 = 0x17

        init_data: list[DmActData] = []

        init_data.append(DmActData(
                    motorType=DM_Motor_Type.DM4340_48V,
                    mode=Control_Mode.POS_VEL_MODE,
                    can_id=canid1,
                    mst_id=mstid1))
        init_data.append(DmActData(
                    motorType=DM_Motor_Type.DM4340_48V,
                    mode=Control_Mode.POS_VEL_MODE,
                    can_id=canid2,
                    mst_id=mstid2))
        init_data.append(DmActData(
                    motorType=DM_Motor_Type.DM4340_48V,
                    mode=Control_Mode.POS_VEL_MODE,
                    can_id=canid3,
                    mst_id=mstid3))
        init_data.append(DmActData(
                    motorType=DM_Motor_Type.DM4340_48V,
                    mode=Control_Mode.POS_VEL_MODE,
                    can_id=canid4,
                    mst_id=mstid4))
        init_data.append(DmActData(
                    motorType=DM_Motor_Type.DM4310_48V,
                    mode=Control_Mode.POS_VEL_MODE,
                    can_id=canid5,
                    mst_id=mstid5))
        init_data.append(DmActData(
                    motorType=DM_Motor_Type.DM4310_48V,
                    mode=Control_Mode.POS_VEL_MODE,
                    can_id=canid6,
                    mst_id=mstid6))
        init_data.append(DmActData(
                    motorType=DM_Motor_Type.DM4310_48V,
                    mode=Control_Mode.POS_VEL_MODE,
                    can_id=canid7,
                    mst_id=mstid7))

        # ── Build can_id -> joint_name lookup from active motors ───────────────
        # Keys must match the joint names published on /joint_states
        self.joint_canid_map: dict[str, int] = {
            'joint_1': canid1,
            'joint_2': canid2,
            'joint_3': canid3,
            'joint_4': canid4,
            'joint_5': canid5,
            'joint_6': canid6,
            'joint_7': canid7,
        }

        # Motors whose direction is inverted relative to the joint_states command
        self.inverted_motors: set[int] = {canid3, canid5}

        self.control = Motor_Control(1000000, 1000000, "E067CA134F67746CCA5451F1BE23BAD8", init_data)

        # Switch all active motors to POS_VEL mode and load a high accel/decel so
        # streamed commands track promptly (anti-jitter, option B).
        for can_id in self.joint_canid_map.values():
            motor = self.control.getMotor(can_id)
            if motor is None:
                continue
            self.control.switchControlMode(motor, Control_Mode_Code.POS_VEL)
            if POS_VEL_ACC > 0.0:
                self.control.change_motor_param(motor, DM_REG.ACC, float(POS_VEL_ACC))
            if POS_VEL_DEC > 0.0:
                self.control.change_motor_param(motor, DM_REG.DEC, float(POS_VEL_DEC))

        if POS_VEL_ACC > 0.0 or POS_VEL_DEC > 0.0:
            self.get_logger().info(
                f'Motor accel set high for prompt tracking: '
                f'ACC={POS_VEL_ACC} DEC={POS_VEL_DEC} rad/s^2'
            )

        self.get_logger().info('PosMotorSub initialised – subscribing to /joint_states')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10,
        )

    def joint_states_callback(self, msg: JointState):
        # Timebase for the velocity estimate. A long gap (startup or a dropped
        # stream) is treated as a restart: reseed the filters, don't derive vel.
        now = time.monotonic()
        dt = 0.0 if self._last_time is None else (now - self._last_time)
        self._last_time = now
        if dt <= 0.0 or dt > STALE_DT_S:
            dt = 0.0

        for i, name in enumerate(msg.name):
            if name not in self.joint_canid_map:
                continue

            can_id = self.joint_canid_map[name]
            motor = self.control.getMotor(can_id)
            if motor is None:
                continue

            raw_pos = msg.position[i] if i < len(msg.position) else 0.0
            if can_id in self.inverted_motors:
                raw_pos = -raw_pos

            # (1) EMA low-pass the target position to reject stream micro-noise.
            pos_prev = self._pos_f.get(can_id)
            if pos_prev is None:
                pos_f = raw_pos                       # seed on first sample
            else:
                pos_f = POS_FILTER_ALPHA * raw_pos + (1.0 - POS_FILTER_ALPHA) * pos_prev

            # (2) Velocity cap. `demand` is the per-cycle position step / dt — the
            #     speed the motor actually needs to reach pos_f this tick. The EMA
            #     (vel_f) is kept only as a smoothed floor; the LIMIT we send is
            #     max(floor, demand * VEL_HEADROOM) so POS_VEL never throttles below
            #     the trajectory and the reach joints stay coordinated on curves.
            if dt > 0.0 and pos_prev is not None:
                demand = abs(pos_f - pos_prev) / dt
                vel_f = (VEL_FILTER_ALPHA * demand
                         + (1.0 - VEL_FILTER_ALPHA) * self._vel_f.get(can_id, 0.0))
            else:
                demand = 0.0
                vel_f = self._vel_f.get(can_id, 0.0)

            self._pos_f[can_id] = pos_f
            self._vel_f[can_id] = vel_f

            # (3) Deadband: while essentially holding, don't re-command — stops
            #     the motor hunting around the setpoint. Small moves accumulate
            #     against the last *sent* position, so slow motion still advances.
            last_sent = self._last_sent.get(can_id)
            if last_sent is not None and abs(pos_f - last_sent) < POS_DEADBAND_RAD:
                continue
            self._last_sent[can_id] = pos_f

            target_vel = min(MAX_VEL, max(MIN_VEL, vel_f, demand * VEL_HEADROOM))
            self.control.control_pos_vel(motor, pos_f, target_vel)

            self.get_logger().debug(
                f'{name}  can_id=0x{can_id:02X}  pos={pos_f:.4f} rad  vel={target_vel:.4f} rad/s'
            )

    def destroy_node(self):
        self.get_logger().info('Shutting down – disabling all motors')
        self.control.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PosMotorSub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()