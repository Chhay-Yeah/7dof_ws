# launch/draw_and_execute.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='arm_bot', executable='drawing_ui_node.py',
             name='drawing_ui_node', output='screen'),
        Node(package='arm_bot', executable='drawing_trajectory_planner.py',
             name='drawing_trajectory_planner', output='screen'),
        Node(package='arm_bot', executable='drawing_executor_node.py',
             name='drawing_executor_node', output='screen'),
        Node(package='arm_bot', executable='ik_arm_v3.py',
             name='ik_7dof_v3', output='screen',
             parameters=[{'tip_link': 'ee'}]),
        Node(package='arm_bot', executable='ik_to_trajectory.py',
             name='ik_to_trajectory', output='screen'),
    ])