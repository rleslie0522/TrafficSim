from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="CS4048_GroupProject",
            executable="traffic",
            output="screen",
            name="PYTraffic"
        )
    ])