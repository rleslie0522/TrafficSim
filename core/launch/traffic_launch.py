# ========================================================================================
#
# NAME:         traffic_launch.py
# DESCRIPTION:  Launch file description for running ROS2 Trafficsim Package.
# AUTHORS:      CALVIN EARNSHAW, FAVOUR JAM, KACPER KOMNATA, REBEKAH LESLIE, ANDREAS MAITA
#
# ========================================================================================


# ========================================================================================
#
# IMPORT DEPENDENCIES
#
# ========================================================================================

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess


# ========================================================================================
#
# LAUNCH DESCRIPTION DEFINITION
#
# ========================================================================================

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="trafficsim",
            output="screen",
            executable="scheduler"
        ),
        ExecuteProcess(
            cmd=[
                "ros2 run pyrobosim_ros prs_worldinit.py"
            ],
            shell=True
        )
    ])
