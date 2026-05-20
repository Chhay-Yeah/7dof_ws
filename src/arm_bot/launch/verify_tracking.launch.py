#!/usr/bin/env python3
"""One-command IK tracking-error test rig.

Launches the full IK pipeline, the Tier 2 cartesian generator, the verifier
node, optionally rqt_plot, and (optionally) auto-fires one demo moveL so the
user immediately sees data flowing.

    ros2 launch arm_bot verify_tracking.launch.py
    ros2 launch arm_bot verify_tracking.launch.py with_plot:=false
    ros2 launch arm_bot verify_tracking.launch.py auto_fire:=false
    ros2 launch arm_bot verify_tracking.launch.py use_gazebo:=false   # fast no-sim path
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    OpaqueFunction, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _setup(context, *_, **__):
    use_gazebo = LaunchConfiguration("use_gazebo").perform(context).lower() == "true"
    with_plot  = LaunchConfiguration("with_plot").perform(context).lower() == "true"
    auto_fire  = LaunchConfiguration("auto_fire").perform(context).lower() == "true"
    delay      = float(LaunchConfiguration("delay").perform(context))

    pkg = FindPackageShare("arm_bot")
    base = "ik_gazebo.launch.py" if use_gazebo else "ik_rvize_test.launch.py"

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", base])
        )
    )

    cartesian = Node(
        package="arm_bot",
        executable="cartesian_path.py",
        name="cartesian_path",
        output="screen",
    )

    verifier = Node(
        package="arm_bot",
        executable="ik_verifier.py",
        name="ik_verifier",
        output="screen",
    )

    plot = ExecuteProcess(
        cmd=[
            "ros2", "run", "rqt_plot", "rqt_plot",
            "/ee_tracking_error/position_error_mm/data",
            "/ee_tracking_error/orientation_error_deg/data",
        ],
        output="log",
        condition=IfCondition(LaunchConfiguration("with_plot")),
    )

    fire_mode = LaunchConfiguration("fire_mode").perform(context)
    fire = ExecuteProcess(
        cmd=["ros2", "run", "arm_bot", "send_test_goal.py",
             "--ros-args", "-p", f"mode:={fire_mode}"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("auto_fire")),
    )

    bag_path = LaunchConfiguration("bag_path").perform(context)
    bag = ExecuteProcess(
        cmd=["ros2", "bag", "record",
             "-o", bag_path,
             "/ee_tracking_error/vector",
             "/ee_target",
             "/ee_pose"],
        output="log",
        condition=IfCondition(LaunchConfiguration("with_bag")),
    )

    actions = [base_launch]
    actions.append(TimerAction(period=delay,           actions=[cartesian, verifier]))
    if with_plot:
        actions.append(TimerAction(period=delay + 1.0, actions=[plot]))
    actions.append(TimerAction(period=delay + 0.5, actions=[bag]))
    if auto_fire:
        actions.append(TimerAction(period=delay + 3.0, actions=[fire]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_gazebo", default_value="true",
                              description="true=Gazebo+full IK; false=ik_rvize_test (no sim, faster)"),
        DeclareLaunchArgument("with_plot",  default_value="true",
                              description="open rqt_plot on the tracking-error topics"),
        DeclareLaunchArgument("auto_fire",  default_value="true",
                              description="auto-publish one moveL goal so data appears immediately"),
        DeclareLaunchArgument("delay",      default_value="12.0",
                              description="seconds to wait before starting cartesian_path / verifier"),
        DeclareLaunchArgument("fire_mode",  default_value="single",
                              description="auto_fire mode: 'single' or 'sequence' (6-move benchmark)"),
        DeclareLaunchArgument("with_bag",   default_value="false",
                              description="record /ee_tracking_error /ee_target /ee_pose to a rosbag"),
        DeclareLaunchArgument("bag_path",   default_value="/tmp/ik_run",
                              description="output path for the rosbag (overwritten each launch)"),
        OpaqueFunction(function=_setup),
    ])
