#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints,
                              PositionConstraint, OrientationConstraint,
                              BoundingVolume, MoveItErrorCodes)
from shape_msgs.msg import SolidPrimitive

_GROUP      = 'arm'
_FRAME      = 'base_link'
_EE_LINK    = 'ee'


class EEBridge(Node):
    def __init__(self):
        super().__init__('moveit_ee_bridge')
        self._busy   = False
        self._client = ActionClient(self, MoveGroup, '/move_action')
        self._sub    = self.create_subscription(PoseStamped, '/ee_target', self.cb, 10)
        self._client.wait_for_server()
        self.get_logger().info('Ready — publish to /ee_target to move the robot')

    def cb(self, msg: PoseStamped):
        if self._busy:
            self.get_logger().warn('Goal in progress — ignoring new target')
            return

        p = msg.pose.position
        self.get_logger().info(f'Got target: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')

        req = MotionPlanRequest()
        req.group_name                      = _GROUP
        req.num_planning_attempts           = 10
        req.allowed_planning_time           = 5.0
        req.max_velocity_scaling_factor     = 0.1
        req.max_acceleration_scaling_factor = 0.1

        pos_c                    = PositionConstraint()
        pos_c.header.frame_id   = _FRAME
        pos_c.link_name         = _EE_LINK
        pos_c.weight            = 1.0

        sp            = SolidPrimitive()
        sp.type       = SolidPrimitive.SPHERE
        sp.dimensions = [0.01]

        bv = BoundingVolume()
        bv.primitives.append(sp)
        bv.primitive_poses.append(msg.pose)
        pos_c.constraint_region = bv

        ori_c                          = OrientationConstraint()
        ori_c.header.frame_id          = _FRAME
        ori_c.link_name                = _EE_LINK
        ori_c.orientation              = msg.pose.orientation
        ori_c.absolute_x_axis_tolerance = 0.05
        ori_c.absolute_y_axis_tolerance = 0.05
        ori_c.absolute_z_axis_tolerance = 0.05
        ori_c.weight                   = 1.0

        c = Constraints()
        c.position_constraints.append(pos_c)
        c.orientation_constraints.append(ori_c)
        req.goal_constraints.append(c)

        goal         = MoveGroup.Goal()
        goal.request = req

        self._busy  = True
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self.goal_cb)

    def goal_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal REJECTED')
            self._busy = False
            return
        self.get_logger().info('Goal ACCEPTED — moving...')
        handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        self._busy = False
        code = future.result().result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('SUCCESS!')
        else:
            name = next((k for k, v in MoveItErrorCodes.__dict__.items()
                         if not k.startswith('_') and v == code), '?')
            self.get_logger().error(f'FAILED — error code: {code} ({name})')


def main():
    rclpy.init()
    rclpy.spin(EEBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
