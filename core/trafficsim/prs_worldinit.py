#!/usr/bin/env python3


# ========================================================================================
#
# NAME:         prs_worldinit.py
# DESCRIPTION:  Generates Pyrobosim environment, incl. robots, task environment, lines,
#               stations.
# AUTHORS:      CALVIN EARNSHAW, FAVOUR JAM, KACPER KOMNATA, REBEKAH LESLIE, ANDREAS MAITA
#
# ========================================================================================


# ========================================================================================
#
# IMPORT DEPENDENCIES
#
# ========================================================================================

# Import Python Common Classes
import os
import threading
import pathlib
import time
import json

# Import ROS2 Common Library Classes
import rclpy
from rclpy.action import ActionServer, ActionClient

# Import PyProj Transformer - used to perform geometric transformations for station coordinates.
from pyproj import Transformer

# Import PyRoboSim Classes
from pyrobosim.core import Robot, World
from pyrobosim.gui import start_gui
from pyrobosim.navigation import (
    ConstantVelocityExecutor,
    AStarPlanner
)
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from pyrobosim_ros.ros_interface import WorldROSWrapper

# Import Message, Action, Server interfaces.
from std_msgs.msg import String
from pyrobosim_msgs.srv import RequestWorldState
from pyrobosim_msgs.action import FollowPath, PlanPath
from trafficsim_interfaces.srv import TestSrvInterface
from trafficsim_interfaces.action import ExecuteTrainRoute

# ============================================
#
# GLOBAL DEFINITIONS
#
# ============================================
data_folder = get_data_folder()

station_dataset_path = pathlib.Path(__file__).parent.parent.parent.parent.parent.resolve().joinpath("src/trafficsim/station_dataset")
# Retrieve stations from generated coords in station_dataset folder
with open(station_dataset_path.joinpath("RailStationCoords.json"), 'r') as f:
    stations = json.load(f)

# Retrieve rail lines from generated lines in station_dataset folder
with open(station_dataset_path.joinpath('RailLines.json'), 'r') as f:
    lines = json.load(f)

transformer = Transformer.from_crs("EPSG:4326", "EPSG:32630", always_xy=True)

scaling_factor = 0.004

station_coordinates = {
    name: (x * scaling_factor, y * scaling_factor)
    for name, (longitude, latitude) in stations.items()
    for x, y in [transformer.transform(longitude, latitude)]
}

def create_world():
    """
        Creates pyrobosim environment containing Simplified Scotland Rail Network
    """

    # Initialise world environment.
    world = World()

    # Set the location and object metadata
    world.set_metadata(
        locations=os.path.join(data_folder, "example_location_data.yaml"),
        objects=os.path.join(data_folder, "example_object_data.yaml"),
    )

    # Iterate through stations given above and add to pyrobosim environment.
    room_size = 5
    for name, (x, y) in station_coordinates.items():
        footprint = [
            (x - room_size / 2, y - room_size / 2), #bottom left
            (x - room_size / 2, y + room_size / 2), #top left
            (x + room_size / 2, y + room_size / 2), #top right
            (x + room_size / 2, y - room_size / 2)  #bottom right
        ]
        world.add_room(name=name, footprint=footprint, color=[0, 0, 0])


    # Add rail lines connecting stations.
    for name, (start, end) in lines.items():
        world.add_hallway(room_start=start, room_end=end, name=name, width=1.25, color=[0.2, 0.2, 0.2])

    # Initialise A* Path Planner
    path_planner = AStarPlanner(
        world = world,
        grid_resolution = 1.0,
        grid_inflation_radius = 0.1,
        heuristic = "euclidean",
        diagonal_motion = True,
        compress_path = True
    )
    path_planner.latest_path = None

    # Add trains to network.
    robot = Robot(
        name="Scotrail_170401",
        radius=0.5,
        path_executor=ConstantVelocityExecutor(linear_velocity=5.0),
        path_planner=path_planner,
        color='#1e467d',
        pose=Pose(0, 0, 0, 0, 0, 0)
    )
    world.add_robot(robot, loc="Dundee")

    return world


def create_ros_node():
    """Initializes ROS node"""
    rclpy.init()

    world = create_world()

    node = WorldROSWrapper(world)

    node.get_logger().info("World loaded.")

    return node



if __name__ == "__main__":
    node = create_ros_node()

    # Start ROS node in separate thread
    ros_thread = threading.Thread(target=lambda: node.start(wait_for_gui=True))
    ros_thread.start()

    # Start GUI in main thread
    start_gui(node.world)
