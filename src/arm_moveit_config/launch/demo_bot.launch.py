"""All-in-one launch: MoveIt stack + one demo executable.

Pick which demo to autorun with the `node` argument:
    ros2 launch arm_moveit_config demo_bot.launch.py node:=circle_trajectory
    ros2 launch arm_moveit_config demo_bot.launch.py node:=square_trajectory
    ros2 launch arm_moveit_config demo_bot.launch.py node:=cartesian_move
    ros2 launch arm_moveit_config demo_bot.launch.py node:=tf_broadcast
    ros2 launch arm_moveit_config demo_bot.launch.py node:=move
    ros2 launch arm_moveit_config demo_bot.launch.py node:=none   # stack only
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


VALID_NODES = {"circle_trajectory", "square_trajectory", "cartesian_move",
               "text_trajectory", "tf_broadcast", "move", "none"}


def launch_setup(context, *args, **kwargs):
    node_name = LaunchConfiguration("node").perform(context)
    delay = float(LaunchConfiguration("delay").perform(context))
    if node_name not in VALID_NODES:
        raise RuntimeError(
            f"Unknown node '{node_name}'. Valid: {sorted(VALID_NODES)}"
        )

    moveit_config = (
        MoveItConfigsBuilder("arm_bot", package_name="arm_moveit_config")
        .robot_description(file_path="config/arm_bot.urdf.xacro")
        .robot_description_semantic(file_path="config/arm_bot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_path = os.path.join(
        get_package_share_directory("arm_moveit_config"), "config", "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("arm_moveit_config"), "config", "ros2_controllers.yaml"
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        output="screen",
    )

    nodes = [
        rviz_node,
        robot_state_publisher,
        move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ]

    if node_name != "none":
        demo_node = Node(
            package="arm_moveit_config",
            executable=node_name,
            name="demo_node",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        )
        nodes.append(TimerAction(period=delay, actions=[demo_node]))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "node",
            default_value="cartesian_move",
            description=f"Demo executable to launch. One of: {sorted(VALID_NODES)}",
        ),
        DeclareLaunchArgument(
            "delay",
            default_value="6.0",
            description="Seconds to wait before starting the demo node (lets RViz/MoveIt come up).",
        ),
        OpaqueFunction(function=launch_setup),
    ])
