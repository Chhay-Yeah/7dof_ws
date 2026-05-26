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
                 
                 # Tuned via workspace_prober.py: this pose puts the EE at
                 # (+0.36, -0.01, +0.14) m with pen direction (-0.76, -0.36,
                 # -0.54) and yields 243/289 reachable cells in a ±80 mm
                 # paper-frame grid → 100 × 100 mm safe centred square.
                 # joint_5 (index 4) = 0.01 instead of dead-zero / 0.4: even a
                 # tiny PID settling error on real hardware would lift the
                 # pen tip off the paper, so we offset slightly off zero.
                 # joint_7 (index 6) = 1.0 — rolls the wrist so the pen sits
                 # roughly perpendicular to the paper for full contact.
                 # The 10× weight on joint_7 below holds it here throughout
                 # the drawing, so no mid-trajectory wrist rolling is needed.
                 'begin_draw_joints': [0.0, -0.7, 0.0, 1.4, 0.01, 0.0, 1.0],
                 # Virtual pen length beyond the EE link (no real gripper).
                 'pen_offset_mm':         100.0,
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
                 # Paper extents matching the prober's safe centred square
                 # under the horizontal-paper convention. Shrunk from 60 to
                 # 40 mm to keep targets well inside the reachable shell —
                 # the 60 mm edge pushed joint_6 to its +0.262 rad limit and
                 # made the IK branch-jump mid-stroke.
                 'workspace_x_mm':        40.0,
                 'workspace_y_mm':        40.0,
                 # lift_mm = lift distance "away from paper". The planner
                 # negates it internally because paper +Z points INTO the
                 # paper (along pen direction). Always pass a positive value.
                 'lift_mm':               10.0,
                 # Verbose per-waypoint joint log so you can inspect the
                 # joint motion the planner asks for. One log line per
                 # trajectory point (~20–100 per drawing). Set False to
                 # silence.
                 'log_joint_deltas':      True,
                 # Indices of joints to FREEZE at begin_draw. [-1] is the
                 # sentinel for "no joints locked" — all 7 joints free.
                 # Other examples:
                 #   [2, 4]          → lock redundant continuous joints 3,5
                 #   [1, 2, 4, 5]    → lock joints 2/3/5/6 (only 1/4/7 free)
                 # Anything that leaves fewer than 6 active joints makes
                 # the IK under-determined and will produce "IK failed"
                 # for off-centre targets.
                 # No joints locked. Hard-locking joint_7 dropped effective
                 # DOF to 6 and made joint_6 saturate at its tight upper
                 # limit (+0.262) on every off-centre waypoint → IK failed.
                 # Instead we just heavily penalise joint_7 motion below
                 # via joint_weights so it stays near 0.5 without losing
                 # the IK's 7-DOF slack.
                 'locked_joints':         [-1],
                 # Null-space pull toward begin_draw. Each waypoint is re-
                 # seeded from q_begin (see drawing_batch_planner.py IK
                 # loop) — null_k just sets the centring strength once
                 # solve_ik starts iterating. 2.0 matches workspace_prober.
                 'null_k':                2.0,
                 # Per-joint penalty for weighted DLS. Uniform = let the
                 # solver pick the lowest-norm joint motion. Crank a joint
                 # higher (e.g. j_6 to 3.0) only if it consistently lands
                 # at a limit in the trajectory dump.
                 # joint_7 weighted 10× to keep the pen tilt close to 0.5
                 # without hard-locking (which broke IK). The IK still
                 # uses joint_7 if it has no other option, but otherwise
                 # leaves it alone. Bump higher (e.g. 50) to glue it even
                 # tighter, lower if you see "IK failed" on edge targets.
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
