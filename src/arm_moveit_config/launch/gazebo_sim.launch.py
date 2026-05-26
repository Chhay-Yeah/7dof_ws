"""Gazebo-only bringup (no MoveIt): sim + ros2_control + RViz.

Replaces the three-terminal dance of:
    ros2 launch arm_bot gazebo.launch.py
    ros2 launch arm_bot bot_controller.launch.py
    ros2 launch arm_bot rviz.launch.py

With Gazebo, the ign_ros2_control plugin embedded in the URDF hosts its
own controller_manager (loaded with arm_bot/config/arm_robot_controllers.yaml),
so we only spawn the controllers — no separate ros2_control_node.

Usage:
    ros2 launch arm_moveit_config gazebo_sim.launch.py
    ros2 launch arm_moveit_config gazebo_sim.launch.py use_rviz:=false
"""
import os
from os import pathsep
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")

    arm_bot_share = get_package_share_directory("arm_bot")

    # ── Robot description (Gazebo-flavoured: IgnitionSystem hardware) ─────────
    xacro_path = os.path.join(arm_bot_share, "urdf", "arm_bot.urdf.xacro")
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_path, " is_ignition:=True"]),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # ── Ignition Gazebo (empty world) ─────────────────────────────────────────
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=pathsep.join([str(Path(arm_bot_share).parent.resolve())]),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ]),
        launch_arguments={"gz_args": "-r -v 4 empty.sdf"}.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "arm_bot",
            "-x", "0.0", "-y", "0.0", "-z", "0.0",
        ],
    )

    # /clock bridge so every ROS node sees sim time.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    # ── Controller spawners (manager lives inside Gazebo via the URDF plugin) ─
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Sequence the spawners so the Gazebo-hosted controller_manager has fully
    # come up before we ask it to load anything.
    spawn_jsb_after_entity = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    spawn_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # ── Optional RViz ─────────────────────────────────────────────────────────
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("arm_bot"), "rviz", "view.rviz",
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),

        gazebo_resource_path,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        clock_bridge,
        spawn_jsb_after_entity,
        spawn_arm_after_jsb,
        rviz_node,
    ])
