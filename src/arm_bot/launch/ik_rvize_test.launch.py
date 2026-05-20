#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Include your existing RViz/URDF launch
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("arm_bot"),
                "launch",
                "rviz.launch.py",
            ])
        )
    )

    # Relay node: /joint_commands -> /joint_states
    relay_node = Node(
        package="arm_bot",
        executable="relay_node.py",   # if it's installed as an executable
        name="joint_relay",
        output="screen",
    )

    # IK node
    ik_node = Node(
        package="arm_bot",
        executable="ik_arm_v3.py",       # if it's installed as an executable
        name="ik_jacobian_pinv_node",
        output="screen",
    )

    return LaunchDescription([
        rviz_launch,
        relay_node,
        ik_node,
    ])