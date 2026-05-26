import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package directories
    pkg_share = get_package_share_directory("arm_bot")

    # Set to "false" to skip the EE/pen-tip breadcrumb tracer (it tanks
    # Gazebo perf during drawing).
    enable_path_tracer_arg = DeclareLaunchArgument(
        "enable_path_tracer", default_value="true",
        description="Spawn gz_path_tracer to drop EE/pen breadcrumbs in Gazebo.",
    )

    # ──────────────── GZ_SIM_RESOURCE_PATH (minimal – adjust as needed) ────────────────
    model_paths = [
        str(Path(pkg_share).parent.resolve()),  # often useful for local models
    ]
    # Add this line only if you actually need models from panda_description
    # model_paths.append(os.path.join(get_package_share_directory("panda_description"), "models"))

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=pathsep.join(model_paths)
    )

    # ──────────────── Robot description (hardcoded xacro from your package) ────────────────
    xacro_path = os.path.join(pkg_share, "urdf", "arm_bot.urdf.xacro")

    robot_description_content = Command([
        "xacro ", xacro_path,
        " is_ignition:=True"   # keep if your xacro uses this argument
    ])

    robot_description = ParameterValue(robot_description_content, value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    # ──────────────── Gazebo – default (no explicit world → uses Gazebo's default) ────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ]),
        # No gz_args → uses default (usually empty simulation)
        # If you want explicit empty world later, you can add:
        # launch_arguments={"gz_args": "-r -v 4 empty.sdf"}.items()
        launch_arguments={
            'gz_args': '-r -v 4 empty.sdf'   # ← this is the key line
            # If you want even more logging: '-r -v 6 empty.sdf'
        }.items()
    )

    # ──────────────── Spawn your robot ────────────────
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "arm_bot",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.0",
        ]
    )

    # ──────────────── Parameter bridge ────────────────
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            # Add more bridges here when needed (joint states, image_raw, etc.)
        ]
    )

    # Image bridge – uncomment when you need /camera/image_raw in ROS
    # ros_gz_image_bridge = Node(
    #     package="ros_gz_image",
    #     executable="image_bridge",
    #     arguments=["/camera/image_raw"],
    #     output="screen"
    # )

    # ──────────────── In-Gazebo EE / pen-tip path tracer ────────────────
    # Subscribes to /joint_states + /robot_description, runs FK, and drops
    # small coloured sphere "breadcrumbs" along the EE (red) and pen-tip
    # (blue) paths via Gazebo's /world/empty/create service. The /marker
    # service is broken in Fortress so we use entity spawning instead.
    # Keep pen_axis_local / pen_offset_mm in sync with
    # draw_and_execute_batch.launch.py.
    gz_path_tracer = Node(
        package="arm_bot",
        executable="gz_path_tracer.py",
        name="gz_path_tracer",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_path_tracer")),
        parameters=[{
            "use_sim_time":   True,
            # EE +X is the URDF's "along the arm" direction (verified by
            # FK at home pose). Keep in sync with draw_and_execute_batch.
            "pen_axis_local": [1.0, 0.0, 0.0],
            "pen_offset_mm":  100.0,
            # Drop a breadcrumb every 5 mm of motion. Decrease for finer
            # trails, increase if Gazebo gets slow.
            "min_step_mm":    5.0,
            # Per-channel cap. 200 crumbs × 5 mm = ~1 m of trail.
            "max_crumbs":     200,
            "crumb_radius_m": 0.003,
            # `ign` for Ignition Fortress (gz-sim 6); switch to `gz` if you
            # ever upgrade to Garden / Harmonic.
            "gz_cli":         "ign",
            # World name must match the SDF launched in gz_args above
            # (default empty.sdf → world name 'empty').
            "world_name":     "empty",
        }],
    )

    return LaunchDescription([
        enable_path_tracer_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        gz_path_tracer,
        # ros_gz_image_bridge,
    ])