#!/usr/bin/env python3


"""
    PRS_WORLDINIT.PY - CS4048 ROBOTICS


    DESIGNED AND WRITTEN BY GROUP XXX IN 2024
"""




import os
import numpy as np


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


data_folder = get_data_folder()


stations = {
    "ABD_ABERDEEN": (-2.099077142995629, 57.143727934509165),
    "DEE_DUNDEE": (-2.9693476729306725, 56.45798804872731),
    "EDB_EDINBURGH": (-3.18827538288093, 55.9529422357554),
    "GLQ_GLASGOW": (-4.25027196270787, 55.864434017276785),
    "INV_INVERNESS": (-4.223450794177058, 57.47990597863926),
    "FTW_FORTWILLIAM": (-5.106064629824768, 56.820585503958405)
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


    world = World()


    room_size = 10 #room size in meters
    for name, (x, y) in station_coordinates.items():
        footprint = [
            (x - room_size / 2, y - room_size / 2), #bottom left
            (x - room_size / 2, y + room_size / 2), #top left
            (x + room_size / 2, y + room_size / 2), #top right
            (x + room_size / 2, y - room_size / 2)  #bottom right
        ]
        world.add_room(name=name, footprint=footprint, color=[0, 0, 0])
   
    world.add_hallway(room_start="ABD_ABERDEEN", room_end="DEE_DUNDEE", width=2)
    world.add_hallway(room_start="ABD_ABERDEEN", room_end="INV_INVERNESS", width=2)
    world.add_hallway(room_start="DEE_DUNDEE", room_end="EDB_EDINBURGH", width=2)
    world.add_hallway(room_start="DEE_DUNDEE", room_end="GLQ_GLASGOW", width=2)
    world.add_hallway(room_start="DEE_DUNDEE", room_end="INV_INVERNESS", width=2)
    world.add_hallway(room_start="EDB_EDINBURGH", room_end="GLQ_GLASGOW", width=2)
    world.add_hallway(room_start="EDB_EDINBURGH", room_end="INV_INVERNESS", width=2)
    world.add_hallway(room_start="GLQ_GLASGOW", room_end="FTW_FORTWILLIAM", width=2)


    #r1coords = [(100, 100), (100, 0), (0,0)]
    #world.add_room(name="EDB_EDINBURGH", footprint=r1coords, color=[0, 0, 0])


    #robot = Robot(
    #    name="1F37_ABD-EDB",
    #    radius=2,
    #    path_executor=ConstantVelocityExecutor(),
    #)


    #world.add_robot(robot, loc="EDB_EDINBURGH")


    return world


if __name__ == "__main__":
    world = create_world()
    start_gui(world)
   


