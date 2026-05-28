#!/usr/bin/env python3
"""
pendant_backend.launch.py — backend stack driven by the 7dof-pendant GUI.

Brings up everything the teach pendant talks to, but NOT the old standalone
drawing_ui_node (the pendant GUI replaces it):

  • Gazebo + spawned robot
  • ros2_control spawners (joint_state_broadcaster + arm_controller)
  • RViz (toggle with rviz:=false)
  • ik_arm_v3        — /ee_target -> /joint_commands        (Cartesian jog)
  • fk_arm_v3        — /joint_states -> /ee_pose            (status / cart jog)
  • ik_to_trajectory — /joint_commands -> /arm_controller/joint_trajectory
  • drawing_batch_planner — /drawing/strokes -> single JointTrajectory (drawing tab)

Drawing uses the BATCH planner (offline spline fit + offline IK + one
JointTrajectory hand-off), not the live drawing_trajectory_planner — the
live real-time IK chase is not reliable. The batch planner publishes joint
trajectories directly to the controller, so the IK/FK nodes above are only
needed for jog/Cartesian, not for drawing.

The in-Gazebo gz_path_tracer breadcrumb node is disabled here
(enable_path_tracer:=false) because it tanks the simulation's visual
performance.

The pendant launches this via `ros2 launch arm_bot pendant_backend.launch.py`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tip_link = LaunchConfiguration("tip_link")
    base_link = LaunchConfiguration("base_link")
    use_rviz = LaunchConfiguration("rviz")
    use_gazebo = LaunchConfiguration("gazebo")

    pkg = FindPackageShare("arm_bot")

    # Gazebo (and the controller spawners that depend on its embedded
    # controller_manager) are gated on `gazebo:=true`. With gazebo:=false the
    # launch brings up only the IK/FK/drawing nodes — useful when attaching to
    # a separately running sim or real hardware.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "gazebo.launch.py"])
        ),
        # Disable the EE/pen breadcrumb tracer — it tanks sim visual perf.
        launch_arguments={"enable_path_tracer": "false"}.items(),
        condition=IfCondition(use_gazebo),
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=IfCondition(use_gazebo),
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=IfCondition(use_gazebo),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "rviz.launch.py"])
        ),
        condition=IfCondition(use_rviz),
    )

    ik_node = Node(
        package="arm_bot", executable="ik_arm_v3.py", name="ik_7dof_v3",
        output="screen",
        parameters=[{"base_link": base_link, "tip_link": tip_link}],
    )
    fk_node = Node(
        package="arm_bot", executable="fk_arm_v3.py", name="fk_7dof_v3",
        output="screen",
        parameters=[{"base_link": base_link, "tip_link": tip_link}],
    )
    bridge = Node(
        package="arm_bot", executable="ik_to_trajectory.py",
        name="ik_to_trajectory", output="screen",
        parameters=[{"step_horizon_s": 0.08}],
    )

    # Batch drawing pipeline: /drawing/strokes -> offline spline + offline IK
    # -> a single JointTrajectory published straight to the controller. The
    # parameter block is the tuned set from draw_and_execute_batch.launch.py;
    # keep the two in sync if you retune.
    drawing_batch_planner = Node(
        package="arm_bot", executable="drawing_batch_planner.py",
        name="drawing_batch_planner", output="screen",
        parameters=[{
            "use_sim_time": True,
            "begin_draw_joints": [0.0, -0.7, 0.0, 1.4, 0.01, 0.0, 1.0],
            "pen_offset_mm": 100.0,
            "pen_axis_local": [1.0, 0.0, 0.0],
            "move_to_begin_seconds": 4.0,
            "dwell_seconds": 3.0,
            "workspace_x_mm": 40.0,
            "workspace_y_mm": 40.0,
            "lift_mm": 10.0,
            "log_joint_deltas": True,
            "locked_joints": [-1],
            "null_k": 2.0,
            "joint_weights": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 10.0],
            "paper_rotation_deg": 270,
            "paper_mirror_x": False,
            "travel_speed_mm_s": 15.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("base_link", default_value="base_link"),
        DeclareLaunchArgument("tip_link", default_value="ee"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gazebo", default_value="true"),
        gazebo,
        jsb_spawner,
        arm_spawner,
        rviz,
        ik_node,
        bridge,
        fk_node,
        drawing_batch_planner,
    ])
