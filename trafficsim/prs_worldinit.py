#!/usr/bin/env python3


"""
    PRS_WORLDINIT.PY - CS4048 ROBOTICS


    DESIGNED AND WRITTEN BY GROUP XXX IN 2024
"""




import os
import rclpy
import numpy as np
import threading


from pyproj import Transformer


from pyrobosim.core import Robot, World, WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.manipulation import GraspGenerator, ParallelGraspProperties
from pyrobosim.navigation import (
    ConstantVelocityExecutor,
    AStarPlanner,
    PRMPlanner,
    RRTPlanner
)
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose

from pyrobosim_ros.ros_interface import WorldROSWrapper



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


scaling_factor = 1000


station_coordinates = {
    name: (x / scaling_factor, y / scaling_factor)
    for name, (longitude, latitude) in stations.items()
    for x, y in [transformer.transform(longitude, latitude)]
}


def create_world():
    """
        Creates a world.
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
        world.add_hallway(room_start=start, room_end=end, name=name, width=2)
    
    # RRT Path Planner - from demo.py
    planner_config = {
        "world": world,
        "bidirectional": True,
        "rrt_connect": False,
        "rrt_star": True,
        "collision_check_step_dist": 0.025,
        "max_connection_dist": 25.0,
        "rewire_radius": 1.5,
        "compress_path": False,
    }
    path_planner = RRTPlanner(**planner_config)

    # Add trains to network.
    robot = Robot(
        name="Scotrail_170401",
        radius=0.5,
        path_executor=ConstantVelocityExecutor(),
        path_planner=path_planner,
    )
    world.add_robot(robot, loc="ABD_ABERDEEN")
    robot = Robot(
        name="Scotrail_158701",
        radius=0.5,
        path_executor=ConstantVelocityExecutor(),
        path_planner=path_planner,
    )
    world.add_robot(robot, loc="EDB_EDINBURGH")
    robot = Robot(
        name="Scotrail_385xxx",
        radius=0.5,
        path_executor=ConstantVelocityExecutor(),
        path_planner=path_planner,
    )
    world.add_robot(robot, loc="GLQ_GLASGOW")


    return world


def create_ros_node():
    """Initializes ROS node"""
    rclpy.init()
    node = WorldROSWrapper(state_pub_rate=0.1, dynamics_rate=0.01)

    node.get_logger().info("Creating world programmatically.")
    
    world = create_world()

    node.set_world(world)

    return node


if __name__ == "__main__":
    node = create_ros_node()

    # Start ROS node in separate thread
    ros_thread = threading.Thread(target=lambda: node.start(wait_for_gui=True))
    ros_thread.start()

    # Start GUI in main thread
    start_gui(node.world)
