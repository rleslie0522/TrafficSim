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
import asyncio

# Import ROS2 Common Library Classes
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

# Import Message, Server, and Action Interfaces
from rclpy.action import ActionServer
from trafficsim.freight import LocalFreightManager
from trafficsim_interfaces.action import RouteTrain, ServiceRoute, MoveFreight
from trafficsim_interfaces.srv import CRSDepartureLookup, CRSAllDepartures, APIRailServiceLookup
from trafficsim_interfaces.msg import UpdateConnection, SpawnFreight, ClaimFreight, DeliverFreight
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
# ----------------------------------------------------------------------------------------

MOVE_SPEED = 0.1

class TrainController(Node):

    # ------------------------------------------------------------------------------------
    #
    # Class Constructor
    #
    # ------------------------------------------------------------------------------------
    def __init__(self, name: str, id: int, robot: Robot, station_graph: StationGraph, current_station_name: str, speed_mult: float):
        super().__init__(name)

        # Link instance of Robot class to Train object.
        self.robot = robot

        self.id = id

        main_callback_group = ReentrantCallbackGroup()

        # Battery consumption.
        self.battery_usage = 0.01

        # Create /route_train action server - used to move train to a given destination, no stops.
        self.route_train_action = ActionServer(
            self,
            RouteTrain,
            f"{self.get_name()}/route_train",
            self.route_train_callback,
            callback_group=main_callback_group
        )

        # Create /follow_service_timetable server - used to trigger automated movement based on timetable.
        self.service_route_action = ActionServer(
            self,
            ServiceRoute,
            f"{self.get_name()}/follow_service_timetable",
            self.follow_service_timetable_callback,
            callback_group=main_callback_group
        )

        self.move_freight_action = ActionServer(
            self,
            MoveFreight,
            f"{self.get_name()}/move_freight",
            self.move_freight_callback,
            callback_group=main_callback_group
        )

        # Initialise pathfinder using custom path planner.
        self.station_graph = station_graph
        self.local_freight_manager = LocalFreightManager(self.id)
        match station_graph.get_node(current_station_name):
            case None:
                raise ValueError(f"Station {current_station_name} not found in station graph")
            case current_position:
                self.current_position = current_position

        self.connection_claim_publisher = self.create_publisher(UpdateConnection, "connection_claim", 10, callback_group=main_callback_group)
        self.connection_claim_subscriber = self.create_subscription(UpdateConnection, "connection_claim", self.connection_claim_callback, 10, callback_group=main_callback_group)
        self.speed_mult = speed_mult

        self.freight_spawn_subscriber = self.create_subscription(SpawnFreight, "freight_spawn", self.freight_spawn_callback, 10, callback_group=main_callback_group)

        self.freight_claim_publisher = self.create_publisher(ClaimFreight, "claim_freight", 10, callback_group=main_callback_group)
        self.freight_claim_subscriber = self.create_subscription(ClaimFreight, "claim_freight", self.freight_claim_callback, 10, callback_group=main_callback_group)

        self.freight_deliver_publisher = self.create_publisher(DeliverFreight, "freight_delivered", 10, callback_group=main_callback_group)

        self.timed_freight_claim_confirmation = self.create_timer(1.1, self.check_internal_claim, autostart=False)
        self.update_loop = self.create_timer(0.1, self.update)

    def is_busy(self):
        return self.robot.executing_nav or self.local_freight_manager.attempted_claim is not None or self.local_freight_manager.claimed is not None

    def update(self):
        if self.is_busy():
            return
        self.try_claim_best_request()

    def freight_spawn_callback(self, msg: SpawnFreight):
        self.local_freight_manager.add_request(msg)
        if not self.is_busy():
            self.try_claim_request(msg)
        pass

    def try_claim_best_request(self):
        best_request = self.local_freight_manager.get_best_request_for_train(self.station_graph, self.current_position)
        if best_request is None:
            return
        self.try_claim_request(best_request)

    def try_claim_request(self, request: SpawnFreight):
        dest = self.station_graph.get_node(request.location)
        if dest is None:
            self.get_logger().error(f"Invalid freight spawn request: {request}")
            return
        self.local_freight_manager.try_claim_request(request.id)
        msg = ClaimFreight()
        msg.id = request.id
        msg.final = False
        msg.train_id = self.id
        msg.distance = float(self.station_graph.get_dist_between_nodes(self.current_position, dest))
        self.get_logger().info(f"Attempting to claim freight {request.id} at {request.location} with distance {msg.distance}")
        self.freight_claim_publisher.publish(msg)
        self.timed_freight_claim_confirmation.reset()

    def check_internal_claim(self):
        if self.local_freight_manager.attempted_claim is None:
            return
        best_train_id = self.local_freight_manager.get_best_train_for_request(self.local_freight_manager.attempted_claim)
        if best_train_id == self.id:
            request = self.local_freight_manager.get_request(self.local_freight_manager.attempted_claim)
            if request is None:
                return
            self.get_logger().info(f"Publishing final claim for freight {request.id} at {request.location}")
            self.freight_claim_publisher.publish(ClaimFreight(id=request.id, train_id=self.id, final=True))

    def freight_claim_callback(self, msg: ClaimFreight):
        if msg.train_id == self.id:
            if msg.final == True:
                self.confirm_internal_freight_claim()
        if msg.final == True:
            self.local_freight_manager.confirm_external_claim(msg.id)
        self.local_freight_manager.add_external_claim(msg)
        pass

    def confirm_internal_freight_claim(self):
        request = self.local_freight_manager.confirm_internal_claim()
        if request is None:
            self.get_logger().error("Internal claim confirmation failed")
            return
        self.get_logger().info(f"Claimed freight {request.id} at {request.location}")
        self.move_freight(MoveFreight.Goal(id=request.id, location=request.location, destination=request.destination))

    def move_freight_callback(self, goal_handle) -> MoveFreight.Result:
        result = self.move_freight(goal_handle.request)
        if result.arrived:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def move_freight(self, goal: MoveFreight.Goal) -> MoveFreight.Result:
        route_train_goal = RouteTrain.Goal()
        route_train_goal.destination = goal.location
        result = self.route_train(route_train_goal)
        if not result.arrived:
            self.get_logger().error("Failed to move freight")
            return MoveFreight.Result(success=False, arrived=False)
        self.pickup_freight(goal.id)

        drop_off_goal = RouteTrain.Goal()
        drop_off_goal.destination = goal.destination
        result = self.route_train(drop_off_goal)
        if not result.arrived:
            self.get_logger().error("Failed to drop off freight")
            return MoveFreight.Result(success=False, arrived=False)
        self.drop_freight()
        return MoveFreight.Result(success=True, arrived=True)

    def pickup_freight(self, freight_id: int):
        self.get_logger().info(f"Picking up freight {freight_id}")
        freight = self.robot.world.get_object_by_name(str(freight_id))
        self.robot._attach_object(freight)
        pass

    def drop_freight(self):
        old_location = self.robot.location
        self.robot.location = self.robot.world.get_entity_by_name(f"{self.current_position.name}_loc_tabletop")
        self.get_logger().info(f"Dropping off freight at  {self.robot.location.name}")
        self.robot.place_object()
        self.robot.location = old_location
        self.freight_deliver_publisher.publish(DeliverFreight(id=self.local_freight_manager.claimed, destination=self.current_position.name))
        self.local_freight_manager.claimed = None
        pass


    # ------------------------------------------------------------------------------------
    #
    # NAME:         connection_claim_callback
    # DESCRIPTION:  Callback that is executed when a connection claim update message is sent,
    #               updating the connection claim status in the local station graph.
    # PARAMETERS:   msg - an UpdateConnection message
    # RETURNS:      none
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
    # DESCRIPTION:  An higher level internal method to move the train along a path to a given destination.
    # PARAMETERS:   path - a list of Station objects
    # RETURNS:      none
    #
    # ------------------------------------------------------------------------------------
    def _follow_path_to_destination(self, path: list[Station]):
        self.get_logger().info(f"Following path: {[str(p).split("(")[0].strip() for p in path]}")
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
    # DESCRIPTION:  A function to publish an UpdateConnection message to the connection_claim,
    #               topic, updating the connection claim status in local station graphs for all trains.
    # PARAMETERS:   start - a Station object idenfitying the start of the connection
    #               end - a Station object identifying the end of the connection
    #               claimed - a boolean value indicating whether the connection is claimed
    # RETURNS:      none
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
    # NAME:         _move_to_neighboring_station
    # DESCRIPTION:  Low level function moving the train to a neighboring station,
    #               assumes station is valid neighbor of current station
    # PARAMETERS:   station - a Station object to move to
    # RETURNS:      none
    #
    # ------------------------------------------------------------------------------------
    # def move_robot(self, prev_time: float, )

    def _move_to_neighboring_station(self, station: Station):
        self.get_logger().info(f"Moving to station: {station.name}")
        self.update_connection(self.current_position, station, True)

        path_follower = PathFollower(self.robot.get_pose(), station.pose, self.speed_mult)

        prev_time = time.time()

        while not self.robot.get_pose().is_approx(station.pose):
            time.sleep(0.1)
            curr_time = time.time()
            self.robot.set_pose(path_follower.get_next_pose(curr_time - prev_time))
            if self.robot.battery_level > 0:
                self.robot.battery_level -= self.battery_usage
            prev_time = curr_time

        self.update_connection(self.current_position, station, False)
        self.current_position = station
        return True

    # ------------------------------------------------------------------------------------
    #
    # NAME:         route_train_callback
    # DESCRIPTION:  Function called when a /route_train action is received, moving the train
    #               to a given destination.
    # PARAMETERS:   goal_handle - a RouteTrain action goal handle passed by an action client.
    # RETURNS:      a RouteTrain action result object
    #
    # ------------------------------------------------------------------------------------
    def route_train_callback(self, goal_handle) -> RouteTrain.Result:
        # TODO race condition resolution if two trains take the same path at the exact same time
        # this is complex to solve and may not be necessary for the scope of the project
        return self.route_train(goal_handle.request, goal_handle)

    def route_train(self, goal: RouteTrain.Goal, goal_handle=None):
        self.get_logger().info(f"Received goal: {goal.destination}")
        dest_name = goal.destination
        dest_node = self.station_graph.get_node(dest_name)
        if dest_node is None:
            return RouteTrain.Result(success=False)

        path = self.station_graph.get_path_between_nodes(self.current_position, dest_node)
        self._follow_path_to_destination(path)
        if goal_handle:
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
            origin = str(self.current_position.name)

            # Create client for RailTrafficScheduler Next Departure Service.
            next_service = self.create_client(
                CRSDepartureLookup,
                "RailTrafficScheduler/request_next_departure"
            )

            # Request next departure from current location of robot.
            req = CRSDepartureLookup.Request()
            req.origin = str(origin)
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
                if self.robot.battery_level <= 5:
                    # chargin battery takes 7 seconds
                    time.sleep(7)
                    self.robot.battery_level = 100
                else:
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
