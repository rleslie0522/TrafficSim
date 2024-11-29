from pyrobosim.core import Robot
from trafficsim.pathing import StationGraph
from trafficsim.train import TrainController
from rclpy.executors import Executor

class TrainRobot(Robot):
    def __init__(self, name, executor: Executor, station_graph: StationGraph, current_station_name: str, **kwargs):
        super().__init__(name, **kwargs)
        self.startup_node(executor, station_graph, current_station_name)

    def startup_node(self, executor: Executor, station_graph: StationGraph, current_station_name: str):
        self.node = TrainController(self.name, self, station_graph, current_station_name)
        executor.add_node(self.node)
