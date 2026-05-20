#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tip_link = LaunchConfiguration("tip_link")
    base_link = LaunchConfiguration("base_link")

    # Brings up robot_state_publisher (latches /robot_description) + RViz.
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("arm_bot"),
                "launch",
                "rviz.launch.py",
            ])
        )
    )

    # Publishes zero defaults for every URDF joint so RViz doesn't collapse,
    # and merges anything published on /joint_commands into /joint_states.
    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{
            "rate": 30,
            "source_list": ["joint_commands"],
        }],
    )

    fk_node = Node(
        package="arm_bot",
        executable="fk_arm_v3.py",
        name="fk_7dof_v3",
        output="screen",
        parameters=[{
            "base_link": base_link,
            "tip_link":  tip_link,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("base_link", default_value="base_link"),
        DeclareLaunchArgument("tip_link",  default_value="ee"),
        rviz_launch,
        jsp,
        fk_node,
    ])