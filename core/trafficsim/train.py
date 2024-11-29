#!/usr/bin/env python3


# ========================================================================================
#
# NAME:         train.py
# DESCRIPTION:  Handles train control within the PyRoboSim environment.
# AUTHORS:      CALVIN EARNSHAW, FAVOUR JAM, KACPER KOMNATA, REBEKAH LESLIE, ANDREAS MAITA
#
# ========================================================================================


# ========================================================================================
#
# IMPORT DEPENDENCIES
#
# ========================================================================================

# Import Python Common Classes
import time

# Import ROS2 Common Library Classes
from rclpy.node import Node

# Import Message, Server, and Action Interfaces
from rclpy.action import ActionServer
from trafficsim_interfaces.action import RouteTrain
from trafficsim_interfaces.msg import UpdateConnection
from trafficsim.pathing import StationGraph, Station, PathFollower
from pyrobosim.core.robot import Robot

# ========================================================================================
#
# CLASS DEFINITIONS
#
# ========================================================================================


# ----------------------------------------------------------------------------------------
#
# NAME:         TrainController
# DESCRIPTION:  A node to handle train movement via set points (rooms) in the environment.
# PARAMETERS:   none
#
# REFERENCES:   - A modification of the demo.py file contained in PyRoboSim_ROS
#               - https://github.com/sea-bass/pyrobosim/blob/main/pyrobosim_ros/examples/demo.py
#
# ----------------------------------------------------------------------------------------

MOVE_SPEED = 0.1

class TrainController(Node):

    # ------------------------------------------------------------------------------------
    #
    # Class Constructor
    #
    # ------------------------------------------------------------------------------------
    def __init__(self, name: str, robot: Robot, station_graph: StationGraph, current_station_name: str):
        super().__init__(name)

        self.robot = robot
        self.station_graph = station_graph
        match station_graph.get_node(current_station_name):
            case None:
                raise ValueError(f"Station {current_station_name} not found in station graph")
            case current_position:
                self.current_position = current_position

        self.route_train_action = ActionServer(
            self,
            RouteTrain,
            f"{self.get_name()}/route_train",
            self.route_train_callback
        )

        self.connection_claim_publisher = self.create_publisher(UpdateConnection, "connection_claim", 10)
        self.connection_claim_subscriber = self.create_subscription(UpdateConnection, "connection_claim", self.connection_claim_callback, 10)

    def connection_claim_callback(self, msg: UpdateConnection):
        start = self.station_graph.get_node(msg.start)
        end = self.station_graph.get_node(msg.end)
        if start is None or end is None:
            self.get_logger().error(f"Invalid connection claim update: {msg}")
            return
        self.station_graph.get_connection_between_nodes(start, end).claimed = msg.claimed

    def _follow_path_to_destination(self, path: list[Station]):
        self.get_logger().info(f"Following path: {path}")
        self.robot.executing_nav = True
        for station in path[1:]:
            connection = self.station_graph.get_connection_between_nodes(self.current_position, station)
            while connection.claimed:
                self.get_logger().info(f"Connection between {self.current_position.name} and {station.name} is claimed, waiting")
                time.sleep(1.0)
            self._move_to_neighboring_station(station)

        self.robot.executing_nav = False

    def update_connection(self, start: Station, end: Station, claimed: bool):
        msg = UpdateConnection()
        msg.start = start.name
        msg.end = end.name
        msg.claimed = claimed
        self.connection_claim_publisher.publish(msg)

    # Assumes station is valid neighbor of current station
    def _move_to_neighboring_station(self, station: Station):
        self.get_logger().info(f"Moving to station: {station.name}")
        self.update_connection(self.current_position, station, True)

        path_follower = PathFollower(self.robot.get_pose(), station.pose)

        prev_time = time.time()

        while not self.robot.get_pose().is_approx(station.pose):
            time.sleep(0.1)
            curr_time = time.time()
            self.robot.set_pose(path_follower.get_next_pose(curr_time - prev_time))
            prev_time = curr_time

        self.update_connection(self.current_position, station, False)
        self.current_position = station
        return True

    def route_train_callback(self, goal_handle):
        # TODO race condition resolution if two trains take the same path at the same time
        self.get_logger().info(f"Received goal: {goal_handle.request}")
        dest_name = goal_handle.request.destination
        dest_node = self.station_graph.get_node(dest_name)
        if dest_node is None:
            goal_handle.abort()
            return RouteTrain.Result(success=False)

        path = self.station_graph.get_path_between_nodes(self.current_position, dest_node)
        self._follow_path_to_destination(path)
        goal_handle.succeed()
        return RouteTrain.Result(success=True, arrived=True)
