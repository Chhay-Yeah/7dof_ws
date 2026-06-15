import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .lib.DM_motor import (
    DM_Motor_Type,
    Control_Mode,
    Control_Mode_Code,
    DmActData,
    Motor_Control,
)

# Velocity limit sent with every position command (rad/s)
DEFAULT_VEL = 5.0
# Velocity floor used when /joint_states reports ~zero (or no) velocity. The old
# behaviour fell back to DEFAULT_VEL (5.0) here, which made the POS_VEL motor
# slam toward a very-close target at full speed every time the commanded velocity
# dithered through zero (start/end of a move, zero-crossings) -> overshoot ->
# correct = jitter, worst on small/slow moves. A small floor lets the motor ease
# to the target instead. Large moves are unaffected (they carry a real nonzero
# velocity and never hit this fallback).
MIN_VEL = 0.5


class PosMotorSub(Node):
    def __init__(self):
        super().__init__('pos_motor_sub')

        self.default_vel = DEFAULT_VEL
        self.min_vel = MIN_VEL   # velocity floor at ~zero commanded velocity

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

        # Switch all active motors to POS_VEL mode
        for can_id in self.joint_canid_map.values():
            motor = self.control.getMotor(can_id)
            if motor is not None:
                self.control.switchControlMode(motor, Control_Mode_Code.POS_VEL)

        self.get_logger().info('PosMotorSub initialised – subscribing to /joint_states')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10,
        )

    def joint_states_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name not in self.joint_canid_map:
                continue

            can_id = self.joint_canid_map[name]
            motor = self.control.getMotor(can_id)
            if motor is None:
                continue

            target_pos = msg.position[i] if i < len(msg.position) else 0.0
            if can_id in self.inverted_motors:
                target_pos = -target_pos
            # Use the real joint velocity when /joint_states provides a nonzero
            # one; at ~zero velocity ease to the target at a small floor instead
            # of slamming at DEFAULT_VEL (the small-move jitter, see MIN_VEL).
            target_vel = (
                abs(msg.velocity[i]) if i < len(msg.velocity) and msg.velocity[i] != 0.0
                else self.min_vel
            )

            self.control.control_pos_vel(motor, target_pos, target_vel)

            self.get_logger().debug(
                f'{name}  can_id=0x{can_id:02X}  pos={target_pos:.4f} rad  vel={target_vel:.4f} rad/s'
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