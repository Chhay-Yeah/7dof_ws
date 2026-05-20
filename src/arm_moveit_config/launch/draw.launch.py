# arm_moveit_config/launch/demo.launch.py
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("arm")
        .robot_description(file_path="config/arm_bot.urdf.xacro")
        .robot_description_semantic(file_path="config/arm_bot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            parameters=[moveit_config.to_dict()],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", "config/moveit.rviz"],
            parameters=[moveit_config.to_dict()],
        ),
    ])