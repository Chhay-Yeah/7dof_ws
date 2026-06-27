# launch/draw_and_execute_batch.launch.py
#
# Batch drawing pipeline:
#   drawing_ui_node          → /drawing/strokes
#   drawing_batch_planner    → /arm_controller/joint_trajectory
#
# Per-stroke spline fit + offline IK + single JointTrajectory hand-off.
# The JointTrajectoryController interpolates at 100 Hz on its own — no
# real-time IK chase, no ik_arm_v3/ik_to_trajectory/executor needed.
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='arm_bot', executable='drawing_ui_node.py',
             name='drawing_ui_node', output='screen'),
        Node(package='arm_bot', executable='drawing_batch_planner.py',
             name='drawing_batch_planner', output='screen',
             parameters=[{
                 'use_sim_time': True,


                 'begin_draw_joints': [0.0, -0.4, 0.0, 1.2, 0.0, 0.0, 0.0],
                 # Virtual pen length beyond the EE link (no real gripper).
                 'pen_offset_mm':         121.0,
                 # This URDF's EE +X is the "along the arm" axis (verified
                 # by FK at home — see workspace_prober output where home
                 # pose has EE +X = (0, 0, +1) = base +Z). At home that's
                 # UP; at begin_draw it tilts to forward-and-down. This is
                 # what your hand-drawn diagram's "Z7" corresponds to —
                 # the URDF designer just labeled the same physical axis
                 # +X instead of +Z in the ee link frame.
                 'pen_axis_local':        [1.0, 0.0, 0.0],
                 # Pre-drawing motion timing.
                 'move_to_begin_seconds': 4.0,
                 'dwell_seconds':         3.0,
                 # Pen-down hold at each stroke's start and end (zero-motion).
                 'stroke_dwell_seconds':  3.0,
                 # Sharp corners: split the spline at vertices turning more than
                 # this (deg); corner_dwell holds there for a hard stop (0=off).
                 'corner_angle_deg':      40.0,
                 'corner_dwell_seconds':  1.0,

                 # lift_mm = lift distance "away from paper". The planner
                 # negates it internally because paper +Z points INTO the
                 # paper (along pen direction). Always pass a positive value.
                 'lift_mm':               -25.0,
                 # Verbose per-waypoint joint log so you can inspect the
                 # joint motion the planner asks for. One log line per
                 # trajectory point (~20–100 per drawing). Set False to
                 # silence.
                 'log_joint_deltas':      True,

                 'locked_joints':         [-1],
                 # Null-space pull toward begin_draw. Each waypoint is re-
                 # seeded from q_begin (see drawing_batch_planner.py IK
                 # loop) — null_k just sets the centring strength once
                 # solve_ik starts iterating. 2.0 matches workspace_prober.
                 'null_k':                2.0,

                 #                          j_1  j_2  j_3  j_4  j_5  j_6  j_7
                 'joint_weights':         [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 10.0],
                 # Canvas axis convention chosen by the user:
                 #   canvas +X (right) → workspace +X (robot's right) → base -Y
                 #   canvas +Y (up)    → workspace +Y (forward)       → base +X
                 # This makes the canvas and the drawing workspace share
                 # the same handedness (both "right = +X, up = +Y"), so
                 # an arrow drawn pointing up on canvas comes out as an
                 # arrow pointing AWAY from the robot (forward) instead
                 # of toward itself.
                 # Matrix is [[0,1,0],[-1,0,0],[0,0,1]] — pure 270° rotation
                 # around base +Z, no mirror.
                 'paper_rotation_deg':    270,
                 'paper_mirror_x':        False,
                 # Between-stroke travel speed (mm/s). Set equal to
                 # draw_speed_mm_s so multi-stroke drawings move at one
                 # steady pace; raise it (e.g. 100) if you want fast
                 # repositioning between distant strokes.
                 'travel_speed_mm_s':     15.0,
             }]),
    ])
