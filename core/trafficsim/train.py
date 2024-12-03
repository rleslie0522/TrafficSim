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
from trafficsim_interfaces.action import RouteTrain, ServiceRoute
from trafficsim_interfaces.srv import CRSDepartureLookup, CRSAllDepartures, APIRailServiceLookup
from trafficsim_interfaces.msg import UpdateConnection
from trafficsim.pathing import StationGraph, Station, PathFollower
from pyrobosim.core.robot import Robot

from datetime import datetime

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
    def __init__(self, name: str, robot: Robot, station_graph: StationGraph, current_station_name: str, speed_mult: float):
        super().__init__(name)

        self.robot = robot
        self.battery_usage = 0.01
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

        self.service_route_action = ActionServer(
            self,
            ServiceRoute,
            f"{self.get_name()}/follow_service_timetable",
            self.follow_service_timetable_callback
        )

        self.connection_claim_publisher = self.create_publisher(UpdateConnection, "connection_claim", 10)
        self.connection_claim_subscriber = self.create_subscription(UpdateConnection, "connection_claim", self.connection_claim_callback, 10)
        self.speed_mult = speed_mult

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

        path_follower = PathFollower(self.robot.get_pose(), station.pose, self.speed_mult)

        prev_time = time.time()

        while not self.robot.get_pose().is_approx(station.pose):
            time.sleep(0.1)
            curr_time = time.time()
            self.robot.set_pose(path_follower.get_next_pose(curr_time - prev_time))
            self.robot.battery_level -= self.battery_usage
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
    
    async def follow_service_timetable_callback(self, goal_handle):
        while True:
            current_location = str(self.robot.location).replace('Room: ', "")

            next_service = self.create_client(
                CRSDepartureLookup,
                "RailTrafficScheduler/request_next_departure"
            )

            req = CRSDepartureLookup.Request()
            req.origin = str(current_location)
            req.lookup_only = False

            future = next_service.call_async(req)
            response = await future

            if response.api_status_code == "999":
                self.get_logger().info("No further services to run from this station - train will remain.")
                break

            service_details = self.create_client(
                APIRailServiceLookup,
                "RailTrafficScheduler/request_service_details"
            )

            service_uid = str(response.service_uid)

            req = APIRailServiceLookup.Request()
            req.year = datetime.today().strftime('%Y')
            req.month = datetime.today().strftime('%m')
            req.date = datetime.today().strftime('%d')
            req.service_uid = service_uid

            future = service_details.call_async(req)
            response = await future

            stops = response.stops

            self.get_logger().info(f"Service ID {req.service_uid} running with {self.get_name()}. Destination: {response.destination}.")

            failed_routing_attempts = 0

            for stop in stops:
                self.get_logger().info("Waiting 5 seconds before moving to next stop...")
                time.sleep(5)
                stop_node_name = stop.replace(" ", "_")
                stop_node = self.station_graph.get_node(stop_node_name)
                if stop_node is None:
                    failed_routing_attempts += 1
                    if failed_routing_attempts == 3:
                        self.get_logger().warn(f"Failed to complete routing to {response.destination} - more than three stops on the route do not exist. Aborting...")
                        # Logic here to move train back to destination?
                    # Attempt to route to next stop in service line.
                    continue
                path = self.station_graph.get_path_between_nodes(self.current_position, stop_node)
                self._follow_path_to_destination(path)
                self.get_logger().info(f"Train arrived at: {stop}. Destination: {response.destination}.")
            
            self.get_logger().info(f"Train has terminated at {response.destination}. Waiting 10 seconds then attempting to find a new route...")

        
        goal_handle.succeed()

        return ServiceRoute.Result(success=True)
