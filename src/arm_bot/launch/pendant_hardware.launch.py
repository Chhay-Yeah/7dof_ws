#!/usr/bin/env python3
"""
pendant_hardware.launch.py — REAL-HARDWARE backend for the 7dof pendant.

Same control stack the pendant already uses, but with the DaMiao-CAN motors
instead of Gazebo:

  • robot_state_publisher  — URDF (for /robot_description, TF, RViz)
  • arm_hw_bridge (arm_bot_hw) — owns the U2CANFD adapter: publishes /joint_states
        from the LIVE encoders and drives the motors (POS_VEL) from
        /arm_controller/joint_trajectory. SAFE START: reads + holds the current
        pose, only moves on command, capped velocity.
  • ik_arm_v3 / fk_arm_v3 / ik_to_trajectory — Cartesian jog + status
  • drawing_batch_planner — drawing tab

There is intentionally NO go_to_start here: on hardware the arm must not auto-move
on launch. Run this, then open the pendant with **Simulation OFF**.

  ros2 launch arm_bot pendant_hardware.launch.py hw_sn:=<your-adapter-serial>

Find your adapter serial with:  ros2 run arm_bot_hw dev_sn
Safety: keep a physical E-stop; verify the "SAFE START — holding current pose"
log matches the real arm before you jog; start with a low hw_max_vel.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tip_link = LaunchConfiguration("tip_link")
    base_link = LaunchConfiguration("base_link")
    use_rviz = LaunchConfiguration("rviz")
    pkg = FindPackageShare("arm_bot")

    # Hardware runs on wall time (no Gazebo /clock).
    sim_time = False

    robot_description = ParameterValue(
        Command(["xacro ", PathJoinSubstitution([pkg, "urdf", "arm_bot.urdf.xacro"]),
                 " is_ignition:=True"]),
        value_type=str)
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": sim_time}],
    )

    # The hardware interface: /arm_controller/joint_trajectory -> DaMiao motors,
    # /joint_states <- live encoders. SAFE START + capped velocity.
    hw_bridge = Node(
        package="arm_bot_hw", executable="hw_bridge", name="arm_hw_bridge",
        output="screen",
        parameters=[{
            "sn": LaunchConfiguration("hw_sn"),
            "max_vel": ParameterValue(LaunchConfiguration("hw_max_vel"), value_type=float),
            "publish_rate": ParameterValue(LaunchConfiguration("hw_rate"), value_type=float),
            "feedback": LaunchConfiguration("hw_feedback"),
        }],
    )

    # ── pendant control nodes (identical params to pendant_backend.launch.py) ──
    ik_node = Node(
        package="arm_bot", executable="ik_arm_v3.py", name="ik_7dof_v3",
        output="screen",
        parameters=[{"base_link": base_link, "tip_link": tip_link,
                     "use_sim_time": sim_time,
                     "tol_pos": 0.002, "tol_rot": 0.01,
                     "inner_iters": ParameterValue(LaunchConfiguration("jog_inner_iters"), value_type=int),
                     "position_only": ParameterValue(LaunchConfiguration("jog_position_only"), value_type=bool),
                     "rot_gain": ParameterValue(LaunchConfiguration("jog_rot_gain"), value_type=float),
                     "null_k": ParameterValue(LaunchConfiguration("jog_null_k"), value_type=float),
                     "null_damp_k": ParameterValue(LaunchConfiguration("jog_null_damp"), value_type=float),
                     "w_thresh": ParameterValue(LaunchConfiguration("jog_w_thresh"), value_type=float),
                     "lambda_sing": ParameterValue(LaunchConfiguration("jog_lambda_sing"), value_type=float),
                     "dq_max": ParameterValue(LaunchConfiguration("jog_dq_max"), value_type=float),
                     #            j1     j2    j3    j4     j5     j6     j7
                     "soft_q_min": [-3.14, -1.6, -3.14, -1.6, -1.6, -0.48, -1.6],
                     "soft_q_max": [ 3.14,  1.6,  3.14,  1.6,  1.6,  0.26,  1.6]}],
    )
    fk_node = Node(
        package="arm_bot", executable="fk_arm_v3.py", name="fk_7dof_v3",
        output="screen",
        parameters=[{"base_link": base_link, "tip_link": tip_link, "use_sim_time": sim_time}],
    )
    ik_to_traj = Node(
        package="arm_bot", executable="ik_to_trajectory.py", name="ik_to_trajectory",
        output="screen",
        parameters=[{"step_horizon_s": 0.08, "use_sim_time": sim_time}],
    )
    ee_marker = Node(
        package="arm_bot", executable="ee_target_marker.py", name="ee_target_marker",
        output="screen",
        parameters=[{"base_link": base_link, "use_sim_time": sim_time}],
        condition=IfCondition(LaunchConfiguration("enable_ee_marker")),
    )
    drawing_batch_planner = Node(
        package="arm_bot", executable="drawing_batch_planner.py", name="drawing_batch_planner",
        output="screen",
        parameters=[{
            "use_sim_time": sim_time,
            "begin_draw_joints": [0.0, -0.7, 0.0, 1.4, 0.01, 0.0, 1.0],
            "pen_offset_mm": 100.0, "pen_axis_local": [1.0, 0.0, 0.0],
            "move_to_begin_seconds": 4.0, "dwell_seconds": 3.0,
            "workspace_x_mm": 40.0, "workspace_y_mm": 40.0, "lift_mm": 0.0,
            "log_joint_deltas": True, "locked_joints": [-1], "null_k": 2.0,
            "joint_weights": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 10.0],
            "paper_rotation_deg": 270, "paper_mirror_x": False, "travel_speed_mm_s": 15.0,
        }],
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, "launch", "rviz.launch.py"])),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("base_link", default_value="base_link"),
        DeclareLaunchArgument("tip_link", default_value="ee"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="Open RViz (needs a display)."),
        DeclareLaunchArgument("enable_ee_marker", default_value="false"),
        # ── hardware ──
        DeclareLaunchArgument("hw_sn", default_value="E067CA134F67746CCA5451F1BE23BAD8",
                              description="U2CANFD adapter serial. Find with `ros2 run arm_bot_hw dev_sn`."),
        DeclareLaunchArgument("hw_max_vel", default_value="1.5",
                              description="Velocity cap [rad/s] sent to every motor. Start low."),
        DeclareLaunchArgument("hw_rate", default_value="50.0",
                              description="/joint_states + command rate [Hz]."),
        DeclareLaunchArgument("hw_feedback", default_value="encoder",
                              choices=["encoder", "command"],
                              description="/joint_states from live encoders (closed-loop) "
                                          "or echo of the last command (open-loop fallback)."),
        # ── jog IK tuning (same defaults as the sim backend) ──
        DeclareLaunchArgument("jog_null_k", default_value="0.0"),
        DeclareLaunchArgument("jog_w_thresh", default_value="0.04"),
        DeclareLaunchArgument("jog_lambda_sing", default_value="0.08"),
        DeclareLaunchArgument("jog_dq_max", default_value="0.05"),
        DeclareLaunchArgument("jog_inner_iters", default_value="2"),
        DeclareLaunchArgument("jog_null_damp", default_value="2.0"),
        DeclareLaunchArgument("jog_rot_gain", default_value="1.0"),
        DeclareLaunchArgument("jog_position_only", default_value="true"),
        rsp,
        hw_bridge,
        ik_node,
        fk_node,
        ik_to_traj,
        ee_marker,
        drawing_batch_planner,
        rviz,
    ])
