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
# REFERENCES:   - TODO
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

        # Link instance of Robot class to Train object.
        self.robot = robot

        # Battery consumption.
        self.battery_usage = 0.01

        # Create /route_train action server - used to move train to a given destination, no stops.
        self.route_train_action = ActionServer(
            self,
            RouteTrain,
            f"{self.get_name()}/route_train",
            self.route_train_callback
        )

        # Create /follow_service_timetable server - used to trigger automated movement based on timetable.
        self.service_route_action = ActionServer(
            self,
            ServiceRoute,
            f"{self.get_name()}/follow_service_timetable",
            self.follow_service_timetable_callback
        )

        # Initialise pathfinder using custom path planner.
        self.station_graph = station_graph
        match station_graph.get_node(current_station_name):
            case None:
                raise ValueError(f"Station {current_station_name} not found in station graph")
            case current_position:
                self.current_position = current_position
        
        self.connection_claim_publisher = self.create_publisher(UpdateConnection, "connection_claim", 10)
        self.connection_claim_subscriber = self.create_subscription(UpdateConnection, "connection_claim", self.connection_claim_callback, 10)
        self.speed_mult = speed_mult

    # ------------------------------------------------------------------------------------
    #
    # NAME:         connection_claim_callback
    # DESCRIPTION:  TODO
    # PARAMETERS:   TODO
    # RETURNS:      TODO
    #
    # ------------------------------------------------------------------------------------
    def connection_claim_callback(self, msg: UpdateConnection):
        start = self.station_graph.get_node(msg.start)
        end = self.station_graph.get_node(msg.end)
        if start is None or end is None:
            self.get_logger().error(f"Invalid connection claim update: {msg}")
            return
        self.station_graph.get_connection_between_nodes(start, end).claimed = msg.claimed

    # ------------------------------------------------------------------------------------
    #
    # NAME:         _follow_path_to_destination
    # DESCRIPTION:  TODO
    # PARAMETERS:   TODO
    # RETURNS:      TODO
    #
    # ------------------------------------------------------------------------------------
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

    # ------------------------------------------------------------------------------------
    #
    # NAME:         update_connection
    # DESCRIPTION:  TODO
    # PARAMETERS:   TODO
    # RETURNS:      TODO
    #
    # ------------------------------------------------------------------------------------
    def update_connection(self, start: Station, end: Station, claimed: bool):
        msg = UpdateConnection()
        msg.start = start.name
        msg.end = end.name
        msg.claimed = claimed
        self.connection_claim_publisher.publish(msg)

    # ------------------------------------------------------------------------------------
    #
    # NAME:         update_connection
    # DESCRIPTION:  TODO Assumes station is valid neighbor of current station
    # PARAMETERS:   TODO
    # RETURNS:      TODO
    #
    # ------------------------------------------------------------------------------------
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

    # ------------------------------------------------------------------------------------
    #
    # NAME:         route_train_callback
    # DESCRIPTION:  TODO Assumes station is valid neighbor of current station
    # PARAMETERS:   TODO
    # RETURNS:      TODO
    #
    # ------------------------------------------------------------------------------------
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
    
    # ------------------------------------------------------------------------------------
    #
    # NAME:         follow_service_timetable_callback
    # DESCRIPTION:  Triggers movement of train by requesting routes from the
    #               RailTrafficScheduler node.
    # PARAMETERS:   goal_handle - contains no parameters.
    # RETURNS:      Result object
    #
    # ------------------------------------------------------------------------------------
    async def follow_service_timetable_callback(self, goal_handle):
        # Runs indefinitely until the train cannot find anymore services to run.
        while True:
            # Get current robot location.
            current_location = str(self.robot.location).replace('Room: ', "")
            origin = str(current_location)

            # Create client for RailTrafficScheduler Next Departure Service.
            next_service = self.create_client(
                CRSDepartureLookup,
                "RailTrafficScheduler/request_next_departure"
            )

            # Request next departure from current location of robot.
            req = CRSDepartureLookup.Request()
            req.origin = str(current_location)
            req.lookup_only = False

            future = next_service.call_async(req)
            response = await future

            # If no further services are available from the current station (indicated by status 999),
            # exit the loop and complete the action.
            if response.api_status_code == "999":
                self.get_logger().info("No further services to run from this station - train will remain.")
                break

            # Create service lookup client to retrieve list of intermediate stops.
            service_details = self.create_client(
                APIRailServiceLookup,
                "RailTrafficScheduler/request_service_details"
            )

            # Collect the unique Service ID from service details.
            service_uid = str(response.service_uid)

            # Request service information, given service unique ID and current date.
            req = APIRailServiceLookup.Request()
            req.year = datetime.today().strftime('%Y')
            req.month = datetime.today().strftime('%m')
            req.date = datetime.today().strftime('%d')
            req.service_uid = service_uid

            future = service_details.call_async(req)
            response = await future

            stops = response.stops

            # Output service details to console.
            self.get_logger().info(f"Service ID {req.service_uid} running with {self.get_name()}. Destination: {response.destination}. Calling at: {response.stops}")

            # Define a flag
            failed_route = False

            # Iterate through each intermediate stop in the stops list, and navigate train.
            for stop in stops:
                # Simulate embarking/disembarking.
                self.get_logger().info("Waiting 5 seconds before moving to next stop...")
                time.sleep(5)

                # Used to count failed routing attempts (where stations are missing from the Pyrobosim environment).
                # This will count up to 3 stops ahead before aborting the action.
                failed_routing_attempts = 0

                # Look for station name in current Station_Graph.
                stop_node_name = stop.replace(" ", "_")
                stop_node = self.station_graph.get_node(stop_node_name)

                # If the stop_node doesn't exist, we record this as a 'failed routing attempt'. We attempt to
                # route up to a maximum of the next three stations ahead before assuming the proposed service route
                # does not exist. In which case, we terminate the service, setting the failed_route flag to TRUE.
                if stop_node is None:
                    failed_routing_attempts += 1
                    if failed_routing_attempts > 3:
                        self.get_logger().warn(f"Failed to complete routing to {response.destination} - more than three consecutive stops on this service do not exist. Aborting...")
                        failed_route = True
                        break
                    continue

                # Plan path between robot current location and next stop.
                path = self.station_graph.get_path_between_nodes(self.current_position, stop_node)

                # Execute path.
                self._follow_path_to_destination(path)
                self.get_logger().info(f"Train arrived at: {stop}. Destination: {response.destination}. Remaining stops: {stops[stops.index(stop):]}")
            
            # If the route failed, then we redirect the train back to the originating station to start a new service.
            if failed_route:
                self.get_logger().warn(f"Train redirecting to origin: {str(origin)}...")
                stop_node_name = str(origin).replace(" ", "_")
                stop_node = self.station_graph.get_node(stop_node_name)
                path = self.station_graph.get_path_between_nodes(self.current_position, stop_node)
                self._follow_path_to_destination(path)
                self.get_logger().info(f"Train arrived at: {str(origin)} awaiting new service.")
            
            self.get_logger().info(f"Train has terminated at {response.destination}. Waiting 10 seconds then attempting to find a new route...")
            time.sleep(10)

        # Action is considered succeessful if there are no further services from the current station to run.
        goal_handle.succeed()
        return ServiceRoute.Result(success=True)
