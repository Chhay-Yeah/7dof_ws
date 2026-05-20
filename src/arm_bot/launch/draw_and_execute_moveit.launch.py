from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='arm_bot', executable='drawing_ui_node.py',
             name='drawing_ui_node', output='screen'),
        Node(package='arm_bot', executable='drawing_trajectory_planner.py',
             name='drawing_trajectory_planner', output='screen'),
        Node(package='arm_bot', executable='drawing_executor_node.py',
             name='drawing_executor_node', output='screen',
             parameters=[{'rate_hz': 0.2}]),   # see Gotcha #1 below
        Node(package='arm_bot', executable='moveit_client.py',
             name='moveit_ee_bridge', output='screen'),
    ])
