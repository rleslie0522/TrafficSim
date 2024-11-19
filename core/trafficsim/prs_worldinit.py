#!/usr/bin/env python3


"""
    PRS_WORLDINIT.PY - CS4048 ROBOTICS

    DESIGNED AND WRITTEN BY GROUP XXX IN 2024
"""

# ============================================
#
# IMPORT DEPENDENCIES
#
# ============================================
import os
import rclpy
import numpy as np
import threading

from rclpy.action import ActionServer

from pyproj import Transformer

from pyrobosim.core import Robot, World
from pyrobosim.gui import start_gui
from pyrobosim.gui import PyRoboSimGUI
from pyrobosim.navigation import (
    ConstantVelocityExecutor,
    AStarPlanner
)
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from pyrobosim_ros.ros_interface import WorldROSWrapper

# ============================================
#
# IMPORT INTERFACES
#
# ============================================
from std_msgs.msg import String
from pyrobosim_msgs.srv import RequestWorldState
from trafficsim_interfaces.srv import TestSrvInterface
from trafficsim_interfaces.action import ExecuteTrainRoute


class ExtendedWorldROSWrapper(WorldROSWrapper):
    """
        An extension of the `pyrobosim_ros.ros_interface.WorldROSWrapper` class
        which will allow `trafficsim` to have finer control over the Pyrobosim
        instance.
    """

    def __init__(self, world):
        super().__init__(
            state_pub_rate=0.1,
            dynamics_rate=0.01
        )

        # Accessible at topic /test_topic
        self.test_topic = self.create_subscription(
            String,
            "test_topic",
            self.msg_callback,
            10
        )

        # Accessible at service /hello
        self.test_srv = self.create_service(
            TestSrvInterface,
            'hello',
            self.service_callback
        )

        # Create Action Server to handle train routing.
        self.train_routing_action = ActionServer(
            self,
            ExecuteTrainRoute,
            'execute_train_route',
            self.train_routing_callback
        )

        # Create Service to get world state
        self.world_state_service = self.create_client(
            RequestWorldState,
            'request_world_state'
        )

        self.set_world(world)

    # Output a string published to /test_topic using msg type std_msgs/msg/String with yaml value "{data: 'Hello World'}"
    def msg_callback(self, msg):
        self.get_logger().info(f"Hello World - msg was: {msg.data}")

    # Output what was received in the request body, based on the contents of the trafficsim interface.
    def service_callback(self, request, response):
        response.outputstr = f"I received: {request.inputstr}"
        self.get_logger().info(f"Request input: {request.inputstr}")
        return response

    # Action Server for train routing.
    async def train_routing_callback(self, goal_handle):
        # Use the /request_world_state service to get information about current robots.
        if not self.world_state_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service not available, aborting goal.')
            goal_handle.abort()
            return ExecuteTrainRoute.Result(success=False)
        world_state_request = RequestWorldState.Request()
        world_state_request.robot = goal_handle.request.train_id
        future = self.world_state_service.call_async(world_state_request)
        response = await future

        # Use response from /request_world_state to check if robot name passed in as parameter is valid.
        # If not, then abort this action.
        if goal_handle.request.train_id not in [robot.name for robot in response.state.robots]:
            self.get_logger().error('Train ID not in Pyrobosim environment, aborting routing attempt.')
            goal_handle.abort()
            return ExecuteTrainRoute.Result(success=False)

        # Train name is correct, we now attempt to create the train route specified by the user.
        self.get_logger().info(f'Routing Train ID: {goal_handle.request.train_id} to destination: {goal_handle.request.destination}...')
        self.get_logger().info(f'This train will call at: {', '.join(goal_handle.request.stops)}')

        goal_handle.succeed()

        result = ExecuteTrainRoute.Result(success=True)
        return result

# ============================================
#
# GLOBAL DEFINITIONS
#
# ============================================
data_folder = get_data_folder()

stations = {
    "ABD_ABERDEEN": (-2.099077142995629, 57.143727934509165),
    "DEE_DUNDEE": (-2.9693476729306725, 56.45798804872731),
    "EDB_EDINBURGH": (-3.18827538288093, 55.9529422357554),
    "GLQ_GLASGOW": (-4.25027196270787, 55.864434017276785),
    "PTH_PERTH": (-3.438421688544935, 56.391456953806795),
    "INV_INVERNESS": (-4.223450794177058, 57.47990597863926),
    "FTW_FORTWILLIAM": (-5.106064629824768, 56.820585503958405)
}

depots = {
    "DEPOT_HAYMARKET": (-3.235502, 55.942165)
}

lines = {
    "ECML_DEE_ABD": ("DEE_DUNDEE", "ABD_ABERDEEN"),
    "ECML_EDB_DEE": ("EDB_EDINBURGH", "DEE_DUNDEE"),
    "FIFE_EDB_PTH": ("EDB_EDINBURGH", "PTH_PERTH"),
    "EDIGLA_EDB_GLQ": ("EDB_EDINBURGH", "GLQ_GLASGOW"),
    "HIGHLAND_PTH_INV": ("PTH_PERTH", "INV_INVERNESS"),
    "DEE_PTH_LINE": ("DEE_DUNDEE", "PTH_PERTH"),
    "SCOTCENTRAL_GLQ_PTH": ("GLQ_GLASGOW", "PTH_PERTH"),
    "ABD_INV_LINE": ("ABD_ABERDEEN", "INV_INVERNESS"),
    "WESTHIGHLAND_GLQ_FTW": ("GLQ_GLASGOW", "FTW_FORTWILLIAM")
}

transformer = Transformer.from_crs("EPSG:4326", "EPSG:32630", always_xy=True)

scaling_factor = 0.001

station_coordinates = {
    name: (x * scaling_factor, y * scaling_factor)
    for name, (longitude, latitude) in stations.items()
    for x, y in [transformer.transform(longitude, latitude)]
}

depot_coordinates = {
    name: (x * scaling_factor, y * scaling_factor)
    for name, (longitude, latitude) in depots.items()
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
    room_size = 10
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
        world.add_hallway(room_start=start, room_end=end, name=name, width=2, color=[0.2, 0.2, 0.2])

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
    world.add_robot(robot, loc="EDB_EDINBURGH")

    return world


def create_ros_node():
    """Initializes ROS node"""
    rclpy.init()

    world = create_world()

    node = ExtendedWorldROSWrapper(world)

    node.get_logger().info("World loaded.")

    # node.set_world(world)

    return node



if __name__ == "__main__":
    node = create_ros_node()

    # Start ROS node in separate thread
    ros_thread = threading.Thread(target=lambda: node.start(wait_for_gui=True))
    ros_thread.start()

    # Start GUI in main thread
    start_gui(node.world)
