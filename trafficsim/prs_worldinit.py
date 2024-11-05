"""
    PRS_WORLDINIT.PY - CS4048 ROBOTICS

    DESIGNED AND WRITTEN BY GROUP XXX IN 2024
"""

import os
import numpy as np

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

def create_world():
    """
        Creates a world.
    """

    world = World()

    return world

if __name__ == "__main__":
    world = create_world()
    start_gui(world)