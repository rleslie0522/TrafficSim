#!/usr/bin/env python3


# ========================================================================================
#
# NAME:         train_robot.py
# DESCRIPTION:  Extension of the PyRoboSim Robot class to allow control of the train (robot)
#               from within our trafficsim modules.
# AUTHORS:      CALVIN EARNSHAW, FAVOUR JAM, KACPER KOMNATA, REBEKAH LESLIE, ANDREAS MAITA
#
# ========================================================================================


# ========================================================================================
#
# IMPORT DEPENDENCIES
#
# ========================================================================================

from pyrobosim.core import Robot
from trafficsim.pathing import StationGraph
from trafficsim.train import TrainController
from rclpy.executors import Executor


# ========================================================================================
#
# CLASS DEFINITIONS
#
# ========================================================================================


# ----------------------------------------------------------------------------------------
#
# NAME:         TrainRobot
# DESCRIPTION:  Extension of Robot class connecting TrainController node to robot.
# PARAMETERS:   none
#
# ----------------------------------------------------------------------------------------

class TrainRobot(Robot):
    def __init__(self, name, executor: Executor, station_graph: StationGraph, current_station_name: str, speed_mult: float, **kwargs):
        super().__init__(name, **kwargs)
        self.startup_node(executor, station_graph, current_station_name, speed_mult)

    def startup_node(self, executor: Executor, station_graph: StationGraph, current_station_name: str, speed_mult: float):
        self.node = TrainController(self.name, self, station_graph, current_station_name, speed_mult)
        executor.add_node(self.node)
