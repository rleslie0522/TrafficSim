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
            package='pyrobosim_ros',  # Replace with the actual package name if different
            executable='prs_worldinit.py',  # Replace with the actual executable name for Pyrobosim
            name='pyrobosim',  # Give the node a name
            output='screen',  # Output logs to the screen
            parameters=[{
                'use_sim_time': True  # Enables the use of simulated time if needed
            }]
        )
    ])