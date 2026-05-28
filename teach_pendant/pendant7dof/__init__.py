"""7dof-pendant — a teach pendant GUI for the 7-DOF robotic arm.

The package is a thin, pip-installable launcher + GUI that drives a ROS 2
Humble backend (Gazebo, ros2_control, the custom IK/drawing nodes). The ROS 2
backend source lives in a colcon workspace that this package can bundle,
build, and launch. See ``bootstrap.py`` for how the workspace is located and
``cli.py`` for the launch flow.
"""

__version__ = "0.1.0"
