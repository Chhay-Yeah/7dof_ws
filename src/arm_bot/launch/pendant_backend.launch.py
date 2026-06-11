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
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tip_link = LaunchConfiguration("tip_link")
    base_link = LaunchConfiguration("base_link")
    use_rviz = LaunchConfiguration("rviz")
    mode = LaunchConfiguration("mode")
    jog_null_k = LaunchConfiguration("jog_null_k")
    jog_w_thresh = LaunchConfiguration("jog_w_thresh")
    jog_lambda_sing = LaunchConfiguration("jog_lambda_sing")
    jog_dq_max = LaunchConfiguration("jog_dq_max")
    jog_inner_iters = LaunchConfiguration("jog_inner_iters")
    jog_null_damp = LaunchConfiguration("jog_null_damp")
    jog_rot_gain = LaunchConfiguration("jog_rot_gain")
    jog_position_only = LaunchConfiguration("jog_position_only")

    pkg = FindPackageShare("arm_bot")
    moveit_pkg = FindPackageShare("arm_moveit_config")

    # Two runtime backends, selected by `mode`:
    #   gazebo  — Gazebo + ros2_control spawners (the physics sim). Uses sim
    #             time from /clock.
    #   moveit  — arm_moveit_config demo.launch.py (MoveIt move_group + fake
    #             ros2_control hardware + MoveIt RViz). No Gazebo, no /clock,
    #             so wall time. Lighter; drawing/jog still work because the
    #             fake controller exposes /arm_controller and /joint_states.
    # The IK/FK/bridge/drawing nodes run in BOTH modes.
    is_gazebo = PythonExpression(["'", mode, "' == 'gazebo'"])
    is_moveit = PythonExpression(["'", mode, "' == 'moveit'"])
    # use_sim_time must follow the clock source: True under Gazebo, False
    # under the MoveIt demo (wall time). Stamping sim time on wall-time
    # trajectories schedules them ~decades in the future and the robot never
    # moves — the historical "robot stays still" bug.
    sim_time = ParameterValue(is_gazebo, value_type=bool)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "gazebo.launch.py"])
        ),
        # EE/pen breadcrumb tracer. Defaults OFF (it tanks sim visual perf),
        # so the pendant's normal Simulation toggle is unaffected. Pass
        # enable_path_tracer:=true to draw the breadcrumb path in Gazebo — e.g.
        # to capture the Figure 4.2 screenshot of the traced path.
        launch_arguments={
            "enable_path_tracer": LaunchConfiguration("enable_path_tracer")
        }.items(),
        condition=IfCondition(is_gazebo),
    )

    # MoveIt demo backend (fake hardware + MoveIt RViz). Brings its own
    # robot_state_publisher, controllers and RViz, so the Gazebo include,
    # the controller spawners and the plain RViz are all disabled in this mode.
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_pkg, "launch", "demo.launch.py"])
        ),
        condition=IfCondition(is_moveit),
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=IfCondition(is_gazebo),
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=IfCondition(is_gazebo),
    )

    # Plain RViz only in gazebo mode (moveit mode gets MoveIt's own RViz) and
    # only when rviz:=true.
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "rviz.launch.py"])
        ),
        condition=IfCondition(PythonExpression(
            ["'", mode, "' == 'gazebo' and '", use_rviz, "' == 'true'"]
        )),
    )

    # Live IK for Cartesian jog only (drawing uses the offline batch planner).
    # position_only=true is the default: the jog tracks EE POSITION only and
    # leaves orientation free for the null-space. WHY: joint_6 has a narrow limit
    # [-0.48,0.26] so the wrist physically can't hold orientation while the EE
    # translates — holding it (rot_gain=1) makes the wrist limit-cycle (the
    # joint_6/joint_7 wobble), and the jog joystick only commands XYZ anyway, so
    # constraining orientation buys nothing. Position-only removes the fight: clean
    # XYZ tracking, no wobble; the tool orientation drifts smoothly as a result.
    #  • inner_iters=2, dq_max=0.05 — gentle resolved-rate loop.
    #  • null_damp_k=2 — null-space velocity damping; higher leaks into the task
    #    and kills tracking, so keep modest.
    #  • rot_gain — only used if position_only:=false (then it's the orientation
    #    authority / wobble-vs-drift tradeoff).
    #  • w_thresh/lambda_sing — manipulability damping floor (arm's normal w~0.013).
    #  • null_k=0 — a fixed null-space PULL limit-cycles + overrides Home/E-stop.
    # All exposed as jog_* launch args; override to retune without a rebuild.
    ik_node = Node(
        package="arm_bot", executable="ik_arm_v3.py", name="ik_7dof_v3",
        output="screen",
        parameters=[{"base_link": base_link, "tip_link": tip_link,
                     "use_sim_time": sim_time,
                     "tol_pos": 0.002,
                     "tol_rot": 0.01,
                     "inner_iters": ParameterValue(jog_inner_iters, value_type=int),
                     "position_only": ParameterValue(jog_position_only, value_type=bool),
                     "rot_gain": ParameterValue(jog_rot_gain, value_type=float),
                     "null_k": ParameterValue(jog_null_k, value_type=float),
                     "null_damp_k": ParameterValue(jog_null_damp, value_type=float),
                     "w_thresh": ParameterValue(jog_w_thresh, value_type=float),
                     "lambda_sing": ParameterValue(jog_lambda_sing, value_type=float),
                     "dq_max": ParameterValue(jog_dq_max, value_type=float),
                     # Soft limits filter unwanted IK branches. DEFAULT is now
                     # the FULL URDF range (max reach). To restore the old
                     # wobble-safe behaviour, pin joint_4 POSITIVE by setting its
                     # soft_q_min entry back to 0.05 (keeps the arm ELBOW-UP and
                     # stops the elbow-up/down flip through the straight-arm
                     # singularity) — the trade-off is much less reach. j6 stays
                     # narrow (hardware). j1/j3 continuous -> sampled +-pi.
                     #            j1     j2    j3    j4     j5     j6     j7
                     "soft_q_min": [-3.14, -1.6, -3.14, -1.6, -1.6, -0.48, -1.6],
                     "soft_q_max": [ 3.14,  1.6,  3.14,  1.6,  1.6,  0.26,  1.6]}],
    )
    fk_node = Node(
        package="arm_bot", executable="fk_arm_v3.py", name="fk_7dof_v3",
        output="screen",
        parameters=[{"base_link": base_link, "tip_link": tip_link,
                     "use_sim_time": sim_time}],
    )
    bridge = Node(
        package="arm_bot", executable="ik_to_trajectory.py",
        name="ik_to_trajectory", output="screen",
        parameters=[{"step_horizon_s": 0.08, "use_sim_time": sim_time}],
    )

    # On startup, command the arm to a conditioned ELBOW-UP start pose (rather
    # than pre-posing the spawn): real-hardware style, and it leaves the all-zeros
    # singularity via a joint-space move (no IK) so jog never starts singular.
    go_to_start = Node(
        package="arm_bot", executable="go_to_start.py",
        name="go_to_start", output="screen",
        parameters=[{"use_sim_time": sim_time,
                     "start_joints": [0.0, -0.4, 0.0, 1.2, 0.0, 0.0, 0.0]}],
    )

    # Debug: cheap in-place RViz spheres for commanded /ee_target (red) vs actual
    # /ee_pose (green). Off by default; add a MarkerArray display on /ee_markers
    # in RViz to see where the jog is sending the EE relative to where it can go.
    ee_marker = Node(
        package="arm_bot", executable="ee_target_marker.py",
        name="ee_target_marker", output="screen",
        parameters=[{"base_link": base_link, "use_sim_time": sim_time}],
        condition=IfCondition(LaunchConfiguration("enable_ee_marker")),
    )

    # Batch drawing pipeline: /drawing/strokes -> offline spline + offline IK
    # -> a single JointTrajectory published straight to the controller. The
    # parameter block is the tuned set from draw_and_execute_batch.launch.py;
    # keep the two in sync if you retune.
    drawing_batch_planner = Node(
        package="arm_bot", executable="drawing_batch_planner.py",
        name="drawing_batch_planner", output="screen",
        parameters=[{
            "use_sim_time": sim_time,
            "begin_draw_joints": [0.0, -0.7, 0.0, 1.4, 0.01, 0.0, 1.0],
            "pen_offset_mm": 100.0,
            "pen_axis_local": [1.0, 0.0, 0.0],
            "move_to_begin_seconds": 4.0,
            "dwell_seconds": 3.0,
            "workspace_x_mm": 40.0,
            "workspace_y_mm": 40.0,
            "lift_mm": 0.0,
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
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Open plain RViz (gazebo mode only)."),
        DeclareLaunchArgument(
            "enable_path_tracer", default_value="false",
            description="Draw the EE/pen breadcrumb path in Gazebo (gazebo mode). "
                        "Off by default (perf); set true for the Fig 4.2 screenshot."),
        DeclareLaunchArgument(
            "enable_ee_marker", default_value="false",
            description="Publish cheap in-place RViz spheres on /ee_markers: red "
                        "/ee_target vs green /ee_pose, for debugging jog reach. "
                        "Off by default; near-zero cost (unlike gz path tracer)."),
        DeclareLaunchArgument(
            "mode", default_value="gazebo",
            choices=["gazebo", "moveit"],
            description="Robot backend: 'gazebo' physics sim or 'moveit' "
                        "demo (fake hardware + MoveIt RViz).",
        ),
        # Cartesian-jog IK tuning (defaults = the values the pendant ships with;
        # override on the CLI to tune singularity behaviour without rebuilding).
        DeclareLaunchArgument(
            "jog_null_k", default_value="0.0",
            description="Jog null-space (q_mid centering) gain. Default 0: the "
                        "live jog must converge+deactivate, and a null-space "
                        "pull leaks through the damped projector into a limit "
                        "cycle that never deactivates (joints oscillate, Home/"
                        "E-stop get overridden). Leave 0 unless you know why."),
        DeclareLaunchArgument(
            "jog_w_thresh", default_value="0.04",
            description="Manipulability below which jog damping ramps up "
                        "(singularity-robust); 0 disables. Watch the "
                        "'near-singular: w=…' log to calibrate."),
        DeclareLaunchArgument(
            "jog_lambda_sing", default_value="0.08",
            description="Max extra DLS damping applied at full singularity."),
        DeclareLaunchArgument(
            "jog_dq_max", default_value="0.05",
            description="Per-tick joint-step cap [rad] for jog; smaller = "
                        "gentler / less overshoot."),
        DeclareLaunchArgument(
            "jog_inner_iters", default_value="2",
            description="DLS sub-iterations per tick for the live jog. Low (1-2) "
                        "= gentle resolved-rate; high (8) is for offline IK."),
        DeclareLaunchArgument(
            "jog_null_damp", default_value="2.0",
            description="Null-space velocity damping gain: resists redundant-DoF "
                        "self-motion (joints wobbling while the EE holds). 0 "
                        "disables. Too high leaks into the task and kills tracking."),
        DeclareLaunchArgument(
            "jog_rot_gain", default_value="1.0",
            description="Orientation-tracking authority for jog (1.0 = full). "
                        "Only used when jog_position_only:=false. Lowering reduces "
                        "wrist fighting/wobble but lets orientation drift."),
        DeclareLaunchArgument(
            "jog_position_only", default_value="true",
            description="Cartesian jog tracks POSITION only; orientation is left "
                        "free (null-space). The jog joystick only commands XYZ "
                        "anyway, so nothing is lost — and it removes the wrist "
                        "orientation fight (joint_6/joint_7 wobble) entirely. Set "
                        "false to hold orientation (uses jog_rot_gain)."),
        gazebo,
        moveit_demo,
        jsb_spawner,
        arm_spawner,
        rviz,
        ik_node,
        bridge,
        fk_node,
        go_to_start,
        ee_marker,
        drawing_batch_planner,
    ])
