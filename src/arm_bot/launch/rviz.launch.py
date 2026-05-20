#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ─────────────────────────────────────────────
    # URDF (xacro → robot_description)
    # ─────────────────────────────────────────────
    urdf_xacro_path = PathJoinSubstitution([
        FindPackageShare("arm_bot"),
        "urdf",
        "arm_bot.urdf.xacro"
    ])

    robot_description_content = Command([
        "xacro ",
        urdf_xacro_path
    ])

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str
        )
    }

    # ─────────────────────────────────────────────
    # RViz config
    # ─────────────────────────────────────────────
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("arm_bot"),
        "rviz",
        "view.rviz"
    ])

    return LaunchDescription([

        # Arguments
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true"
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false"
        ),

        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                robot_description,
                {"use_sim_time": use_sim_time}
            ],
        ),

        # REMOVED: joint_state_publisher — your relay_node now publishes /joint_states

        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_config_file],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])