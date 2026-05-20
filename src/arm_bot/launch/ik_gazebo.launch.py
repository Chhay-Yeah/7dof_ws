#!/usr/bin/env python3
"""
ik_gazebo.launch.py — full IK-on-Gazebo validation stack.

Brings up:
  • Gazebo + spawned robot
  • ros2_control + arm_controller + joint_state_broadcaster
  • RViz (optional)
  • ik_arm_v3       — Cartesian IK, /ee_target -> /joint_commands
  • ik_to_trajectory — /joint_commands -> /arm_controller/joint_trajectory
  • fk_arm_v3       — /joint_states -> /ee_pose (for verification)

Drive the arm by publishing a PoseStamped to /ee_target.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tip_link  = LaunchConfiguration("tip_link")
    base_link = LaunchConfiguration("base_link")

    pkg = FindPackageShare("arm_bot")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "gazebo.launch.py"])
        )
    )

    # Gazebo embeds its own controller_manager via gz_ros2_control,
    # so we only need the spawners — NOT a second ros2_control_node.
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "rviz.launch.py"])
        )
    )

    ik_node = Node(
        package="arm_bot",
        executable="ik_arm_v3.py",
        name="ik_7dof_v3",
        output="screen",
        parameters=[{"base_link": base_link, "tip_link": tip_link}],
    )

    fk_node = Node(
        package="arm_bot",
        executable="fk_arm_v3.py",
        name="fk_7dof_v3",
        output="screen",
        parameters=[{"base_link": base_link, "tip_link": tip_link}],
    )

    bridge = Node(
        package="arm_bot",
        executable="ik_to_trajectory.py",
        name="ik_to_trajectory",
        output="screen",
        parameters=[{"step_horizon_s": 0.08}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("base_link", default_value="base_link"),
        DeclareLaunchArgument("tip_link",  default_value="ee"),
        gazebo,
        jsb_spawner,
        arm_spawner,
        rviz,
        ik_node,
        bridge,
        fk_node,
    ])
