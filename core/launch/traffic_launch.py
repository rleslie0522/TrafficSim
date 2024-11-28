from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package="trafficsim",
        #     output="screen",
        #     executable="train"
        # ),
        ExecuteProcess(
            cmd=[
                "ros2 run pyrobosim_ros prs_worldinit.py"
            ],
            shell=True
        )
    ])
