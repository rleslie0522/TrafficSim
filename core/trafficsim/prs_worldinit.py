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
import json
import copy
import asyncio

# Import ROS2 Common Library Classes
import rclpy
import rclpy.node

# Import PyProj Transformer - used to perform geometric transformations for station coordinates.
from pyproj import Transformer

# Import PyRoboSim Classes
from pyrobosim.core import World
from pyrobosim.gui import start_gui
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from pyrobosim_ros.ros_interface import WorldROSWrapper

from rclpy.executors import MultiThreadedExecutor, Executor
from trafficsim.freight import GlobalFreightManager
from trafficsim.train_robot import TrainRobot
from trafficsim.pathing import StationGraph

# ========================================================================================
#
# GLOBAL DEFINITIONS
#
# ========================================================================================

# Link to data folder containing objects in PyRoboSim.
data_folder = get_data_folder()

# Retrieve stations from generated coords in station_dataset folder
station_dataset_path = pathlib.Path(__file__).parent.parent.parent.parent.parent.resolve().joinpath("src/trafficsim/station_dataset")
with open(station_dataset_path.joinpath("RailStationCoords.json"), 'r') as f:
    stations = json.load(f)

# Retrieve rail lines from generated lines in station_dataset folder
with open(station_dataset_path.joinpath('RailLines.json'), 'r') as f:
    lines = json.load(f)

# Retrieve trains from station_dataset folder
with open(station_dataset_path.joinpath('Trains.json'), 'r') as f:
    trains = json.load(f)["trains"]

# Create Transformer object to convert lat/lon into x,y coords - https://gis.stackexchange.com/a/78944
transformer = Transformer.from_crs("EPSG:4326", "EPSG:32630", always_xy=True)

# Define scaling factor used to draw station coordinates in PyRoboSim environment.
# scaling_factor = 0.004
scaling_factor = 0.1

# Create 2D plane station coordinates for PyRoboSim.
station_coordinates = {
    name: (x * scaling_factor, y * scaling_factor)
    for name, (longitude, latitude) in stations.items()
    for x, y in [transformer.transform(longitude, latitude)]
}

# print(station_coordinates)
station_graph = StationGraph(station_coordinates, lines)

# Define train types.
train_types = {
    "43": {
        "operator": "Scotrail",
        "colour": "#1e467d",
        "battery_usage": 0.01,
        "linear_velocity": 10
    },
    "158": {
        "operator": "Scotrail",
        "colour": "#1e467d",
        "battery_usage": 0.02,
        "linear_velocity": 10
    },
    "170": {
        "operator": "Scotrail",
        "colour": "#1e467d",
        "battery_usage": 0.01,
        "linear_velocity": 10
    },
    "801": {
        "operator": "LNER",
        "colour": "#ce0e2d",
        "battery_usage": 0.005,
        "linear_velocity": 10
    }
}


# ========================================================================================
#
# CLASS DEFINITIONS
#
# ========================================================================================

class NavigationOptions:
    # This class serves one purpose - to pass in a battery_usage value to the Robot constructor
    # in the exact format PyRoboSim wants it.
    def __init__(self, battery_usage: float):
        self.battery_usage = battery_usage


# ========================================================================================
#
# METHOD DEFINITIONS
#
# ========================================================================================


# ----------------------------------------------------------------------------------------
#
# NAME:         create_world()
# DESCRIPTION:  Creates a world to import into PyRoboSim.
# PARAMETERS:   none
# RETURNS:      World - a PyRoboSim world instance.
#
# REFERENCES:   - A modification of the demo.py file contained in PyRoboSim_ROS
#               - https://github.com/sea-bass/pyrobosim/blob/main/pyrobosim_ros/examples/demo.py
#
# ----------------------------------------------------------------------------------------

def create_world(executor: Executor):
    # Initialise world environment.
    world = World()

    # Set the location and object metadata
    world.set_metadata(
        locations=os.path.join(data_folder, "example_location_data.yaml"),
        objects=os.path.join(data_folder, "example_object_data.yaml"),
    )

    # Iterate through stations given above and add to pyrobosim environment.
    room_size = 20
    for name, (x, y) in station_coordinates.items():
        footprint = [
            (x - room_size / 2, y - room_size / 2), #bottom left
            (x - room_size / 2, y + room_size / 2), #top left
            (x + room_size / 2, y + room_size / 2), #top right
            (x + room_size / 2, y - room_size / 2)  #bottom right
        ]
        world.add_room(name=name, footprint=footprint, color=[0, 0, 0])
        world.add_location(name=f"{name}_loc", category="table", parent=name, pose=Pose(x, y))

    # Add rail lines connecting stations.
    for name, (start, end) in lines.items():
        world.add_hallway(room_start=start, room_end=end, name=name, width=1.25, color=[0.2, 0.2, 0.2])

    train_id = 0
    # Add trains to network.
    for train in trains:
        robot = TrainRobot(
            name=f"{train["operator"]}_{train["class"]}{train["id"]}",
            id = train_id,
            radius=0.5,
            color=train_types[train["class"]]["colour"],
            pose=Pose(0, 0, 0, 0, 0, 0),
            station_graph=copy.deepcopy(station_graph),
            current_station_name=train["starting_station"],
            executor=executor,
            speed_mult=train_types[train["class"]]["linear_velocity"],
        )
        train_id += 1
        world.add_robot(robot, loc=f"{train["starting_station"]}")
        # workaround for the robot not being placed in the correct location
        starter_location = world.get_entity_by_name(f"{train["starting_station"]}_loc_tabletop")
        robot.location = starter_location
        robot.set_pose(starter_location.pose)

    freight_manager = GlobalFreightManager(20, station_graph, world)
    executor.add_node(freight_manager)
    return world


# ----------------------------------------------------------------------------------------
#
# NAME:         create_ros_node()
# DESCRIPTION:  Initialises a new ROS node responsible for loading world into PyRoboSim GUI.
# PARAMETERS:   none
# RETURNS:      World - a PyRoboSim world instance.
#
# REFERENCES:   - https://github.com/sea-bass/pyrobosim/blob/main/pyrobosim_ros/examples/demo.py
#
# ----------------------------------------------------------------------------------------

def create_ros_node(executor: Executor):
    world = create_world(executor)
    node = WorldROSWrapper(world)
    node.get_logger().info("World loaded.")
    return node


# ----------------------------------------------------------------------------------------
#
# NAME:         threaded_startup()
# DESCRIPTION:  Meant to be called in a separate thread, sets up the wordl node with the
#               executor and starts the ROS node.
# PARAMETERS:   executor - an Executor used to run the ROS node
#               world_node - WorldROSWrapper instance to be run
# RETURNS:      none
#
# ----------------------------------------------------------------------------------------

def threaded_startup(executor: Executor, world_node: WorldROSWrapper):
    world_node.start(True, False)
    executor.add_node(world_node)
    world_node.executor = executor
    try:
        executor.spin()
    finally:
        world_node.shutdown()

# ----------------------------------------------------------------------------------------
#
# NAME:         main
# DESCRIPTION:  The main entry point for this module.
# PARAMETERS:   none
# RETURNS:      none
#
# REFERENCES:   - https://github.com/sea-bass/pyrobosim/blob/main/pyrobosim_ros/examples/demo.py
#
# ----------------------------------------------------------------------------------------

if __name__ == "__main__":
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=os.cpu_count())
    world_node = create_ros_node(executor)

    # Start ROS node in separate thread
    ros_thread = threading.Thread(target=lambda: threaded_startup(executor, world_node))
    ros_thread.start()

    # Start GUI in main thread
    start_gui(world_node.world)
