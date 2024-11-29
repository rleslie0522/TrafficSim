from pyrobosim.core import Robot
from trafficsim.pathing import StationGraph
from trafficsim.train import TrainController
from rclpy.executors import Executor

class TrainRobot(Robot):
    def __init__(self, name, executor: Executor, station_graph: StationGraph, current_station_name: str, speed_mult: float, **kwargs):
        super().__init__(name, **kwargs)
        self.startup_node(executor, station_graph, current_station_name, speed_mult)

    def startup_node(self, executor: Executor, station_graph: StationGraph, current_station_name: str, speed_mult: float):
        self.node = TrainController(self.name, self, station_graph, current_station_name, speed_mult)
        executor.add_node(self.node)
