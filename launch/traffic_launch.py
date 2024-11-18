from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="trafficsim",
            output="screen",
            executable="traffic"
        ),
        Node(
            package='pyrobosim_ros',
            executable='prs_worldinit.py',
            name='pyrobosim',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }]
        )
    ])