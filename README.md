# 7-DOF Drawing Robot

ROS 2 system for replicating user-drawn paths on a 7-DOF robotic arm.

## Architecture
- `drawing_ui_node` — PyQt6 canvas that captures strokes and publishes JSON
- `drawing_trajectory_planner` — converts strokes to Cartesian path with SLERP orientations
- `ik_node` — inverse kinematics solver
- `arm_controller` — FollowJointTrajectory action server

## Build
\`\`\`bash
cd ~/7dof_ws
colcon build
source install/setup.bash
\`\`\`

## Run
\`\`\`bash
ros2 launch robot_drawing draw_and_execute.launch.py
\`\`\`
