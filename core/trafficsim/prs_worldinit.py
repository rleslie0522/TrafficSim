#!/usr/bin/env python3


# ========================================================================================
#
# NAME:         prs_worldinit.py
# DESCRIPTION:  Generates Pyrobosim environment, incl. robots, task environment, lines,
#               stations.
# AUTHORS:      CALVIN EARNSHAW, FAVOUR JAM, KACPER KOMNATA, REBEKAH LESLIE, ANDREAS MAITA
#
# ========================================================================================


# ========================================================================================
#
# IMPORT DEPENDENCIES
#
# ========================================================================================

# Import Python Common Classes
import os
import threading
import pathlib
import time
import json

# Import ROS2 Common Library Classes
import rclpy
from rclpy.action import ActionServer, ActionClient

# Import PyProj Transformer - used to perform geometric transformations for station coordinates.
from pyproj import Transformer

# Import PyRoboSim Classes
from pyrobosim.core import Robot, World
from pyrobosim.gui import start_gui
from pyrobosim.navigation import (
    ConstantVelocityExecutor,
    AStarPlanner
)
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from pyrobosim_ros.ros_interface import WorldROSWrapper

# Import Message, Action, Server interfaces.
from std_msgs.msg import String
from pyrobosim_msgs.srv import RequestWorldState
from pyrobosim_msgs.action import FollowPath, PlanPath
from trafficsim_interfaces.srv import TestSrvInterface
from trafficsim_interfaces.action import ExecuteTrainRoute


# ========================================================================================
#
# NAME:         ExtendedWorldROSWrapper()
# DESCRIPTION:  Used to extend the topics/actions/servers available within the Pyrobosim
#               environment.
# PARAMETERS:   world - an object of datatype pyrobosim.core.World
#
# ========================================================================================

class ExtendedWorldROSWrapper(WorldROSWrapper):

    # ------------------------------------------------------------------------------------
    # Class Constructor
    # ------------------------------------------------------------------------------------
    def __init__(self, world):
        super().__init__(
            state_pub_rate=0.1,     # Set robot_state publish rate to every 100ms.
            dynamics_rate=0.01
        )

        # Create Action Server to handle train routing requests - https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html#writing-an-action-server
        self.train_routing_action = ActionServer(
            self,
            ExecuteTrainRoute,
            'execute_train_route',
            self.train_routing_callback
        )

        # Create Service to get PyRoboSim world state - https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html#write-the-client-node
        self.world_state_service = self.create_client(
            RequestWorldState,
            'request_world_state'
        )

        # Create class-instance variables to define our PyRoboSim PlanPath and FollowPath action clients.
        self.planning_client = None
        self.follow_client = None

        # Set the PyRoboSim world to the contents of the world passed into the constructor.
        self.set_world(world)


    # ------------------------------------------------------------------------------------
    # 
    # NAME:         train_routing_callback()
    # DESCRIPTION:  Handles individual robot routing in PyRoboSim.
    # PARAMETERS:   goal_handle - the contents of the goal callback from ROS 2
    # RETURNS:      ExecuteTrainRoute.Result object, containing the result of the action.
    #
    # REFERENCES:   - https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html
    #               - https://www.velotio.com/engineering-blog/async-features-in-python
    #
    # ------------------------------------------------------------------------------------
    async def train_routing_callback(self, goal_handle):
        # Call the /request_world_state service to get information about trains, lines,
        # and stations in current PyRoboSim environment.
        if not self.world_state_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Cannot retrive current PyRoboSim state - aborting.')
            goal_handle.abort()
            return ExecuteTrainRoute.Result(success=False)
        world_state_request = RequestWorldState.Request()
        world_state_request.robot = goal_handle.request.train_id
        future = self.world_state_service.call_async(world_state_request)
        response = await future

        # Store list of all train IDs currently in PyRoboSim environment.
        trains = [robot.name for robot in response.state.robots]

        # Get initial state information about the train involved in this routing callback.
        current_robot_state = [robot for robot in response.state.robots if robot.name == goal_handle.request.train_id][0]

        # Use response from /request_world_state to check if robot name passed in as
        # parameter is valid. If not, then abort.
        if goal_handle.request.train_id not in trains:
            self.get_logger().error('Train ID not in Pyrobosim environment, aborting routing attempt.')
            goal_handle.abort()
            return ExecuteTrainRoute.Result(success=False)

        # Use response from /request_world_state to check if all stations passed in
        # as parameters exist within the environment. If not, then abort.
        nonexistent_stations = set(goal_handle.request.stops).difference(
            set([hallway.room_start for hallway in response.state.hallways] +
                [hallway.room_end for hallway in response.state.hallways]
            )
        )
        if len(nonexistent_stations) > 0:
            self.get_logger().error('One or more Station IDs in "stops" parameter not in Pyrobosim environment, aborting routing attempt.')
            goal_handle.abort()
            return ExecuteTrainRoute.Result(success=False)

        # Log attempt to create a train route for the Train ID given in the action parameters.
        self.get_logger().info(f'Routing Train ID: {goal_handle.request.train_id} to destination: {goal_handle.request.destination}...')
        self.get_logger().info(f'This train will call at: {', '.join(goal_handle.request.stops[:-1])}, and {goal_handle.request.stops[-1]}')

        # Create two Action Clients - one to communicate with Pyrobosim's Path Planner, the other for Pyrobosim Follow Path action.
        # https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html#writing-an-action-client
        self.planning_client = ActionClient(
            self,
            PlanPath,
            f"{goal_handle.request.train_id}/plan_path"
        )
        self.follow_client = ActionClient(
            self,
            FollowPath,
            f"{goal_handle.request.train_id}/follow_path"
        )

        # Abort the Routing Action if either the PlanPath or FollowPath Action Servers are unavailable.
        if not self.planning_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'{goal_handle.request.train_id} - Cannot initialise planning client - server unavailable. Aborting...')
            goal_handle.abort()
            return ExecuteTrainRoute.Result(success=False)
        if not self.follow_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'{goal_handle.request.train_id} - Cannot initialise follow client - server unavailable. Aborting...')
            goal_handle.abort()
            return ExecuteTrainRoute.Result(success=False)

        # Iterate through each station given in the 'stops' parameter of the action request, call the PlanPath action client from current stop to
        # next stop in the 'stops' array, then follow said path. Based on the 'send_goal' function seen here: 
        # https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html#writing-an-action-server
        for i in range(0, len(goal_handle.request.stops) + 1):
            # Create PlanPath Action Request
            planning_goal = PlanPath.Goal()
            planning_goal.target_location = goal_handle.request.stops[i] if i < len(goal_handle.request.stops) else goal_handle.request.destination

            # Programmatically send 'ros2 action send_goal' command to PlanPath Action Server.
            planning_goal_handle = await self.planning_client.send_goal_async(planning_goal)

            # Abort routing action if the action server rejects the request.
            if not planning_goal_handle.accepted:
                self.get_logger().error(f'{goal_handle.request.train_id} - Cannot plan train path - goal rejected by server. Aborting...')
                goal_handle.abort()
                return ExecuteTrainRoute.Result(success=False)

            self.get_logger().info(f'{goal_handle.request.train_id} - Goal within PlanPath action has been accepted. Waiting for result...')
            planning_result_future = planning_goal_handle.get_result_async()
            planning_result = await planning_result_future

            if not planning_result.result.execution_result.status == 0:
                self.get_logger().error(f'{goal_handle.request.train_id} - Cannot complete PlanPath action - error code {str(planning_result.result.execution_result.status)} returned by server. Aborting...')
                goal_handle.abort()
                return ExecuteTrainRoute.Result(success=False)

            self.get_logger().info(f'{goal_handle.request.train_id} - Route planned successfully.')

            follow_goal = FollowPath.Goal()
            follow_goal.path = planning_result.result.path

            follow_goal_handle = await self.follow_client.send_goal_async(follow_goal)

            if not follow_goal_handle.accepted:
                self.get_logger().error(f'{goal_handle.request.train_id} - Cannot follow train path - goal rejected by server. Aborting...')
                goal_handle.abort()
                return ExecuteTrainRoute.Result(success=False)

            self.get_logger().info(f'{goal_handle.request.train_id} - Goal within FollowPath action has been accepted. Agent moving...')
            follow_result_future = follow_goal_handle.get_result_async()
            follow_result = await follow_result_future

            if not follow_result.result.execution_result.status == 0:
                self.get_logger().error(f'{goal_handle.request.train_id} - Cannot complete FollowPath action - error code {str(follow_result.result.execution_result.status)} returned by server. Aborting...')
                goal_handle.abort()
                return ExecuteTrainRoute.Result(success=False)

            self.get_logger().info(f'{goal_handle.request.train_id} - Path segment traversed successfully.')

            if i < len(goal_handle.request.stops):
                self.get_logger().info(f'{goal_handle.request.train_id} - Stopped at: {goal_handle.request.stops[i]}. Next stop: {goal_handle.request.destination if i == len(goal_handle.request.stops)-1 else goal_handle.request.stops[i+1]}.')
                self.get_logger().info(f'{goal_handle.request.train_id} - Remaining at {goal_handle.request.stops[i]} for 10 seconds.')
                # Simulate embarking and disembarking.
                time.sleep(10)
            else:
                self.get_logger().info(f'{goal_handle.request.train_id} - Stopped at: {goal_handle.request.destination}, where this train will terminate.')
                self.get_logger().info(f'{goal_handle.request.train_id} - Train has reached final destination - path complete.')


        # # TODO: Proof-of-Concept for Action Feedback - Implement this each time the train stops at a designated stop in the list!
        # fb = ExecuteTrainRoute.Feedback()
        # for i in range(0,5):
        #     fb.visited.append('stop' + str(i))
        #     goal_handle.publish_feedback(fb)        # Publishes to a topic /execute_train_route/_action/feedback
        #     time.sleep(1)

        goal_handle.succeed()

        result = ExecuteTrainRoute.Result(success=True)
        return result

# ============================================
#
# GLOBAL DEFINITIONS
#
# ============================================
data_folder = get_data_folder()

station_dataset_path = pathlib.Path(__file__).parent.parent.parent.parent.parent.resolve().joinpath("src/trafficsim/station_dataset")
# Retrieve stations from generated coords in station_dataset folder
with open(station_dataset_path.joinpath("RailStationCoords.json"), 'r') as f:
    stations = json.load(f)

# Retrieve rail lines from generated lines in station_dataset folder
with open(station_dataset_path.joinpath('RailLines.json'), 'r') as f:
    lines = json.load(f)

transformer = Transformer.from_crs("EPSG:4326", "EPSG:32630", always_xy=True)

scaling_factor = 0.004

station_coordinates = {
    name: (x * scaling_factor, y * scaling_factor)
    for name, (longitude, latitude) in stations.items()
    for x, y in [transformer.transform(longitude, latitude)]
}

def create_world():
    """
        Creates pyrobosim environment containing Simplified Scotland Rail Network
    """

    # Initialise world environment.
    world = World()

    # Set the location and object metadata
    world.set_metadata(
        locations=os.path.join(data_folder, "example_location_data.yaml"),
        objects=os.path.join(data_folder, "example_object_data.yaml"),
    )

    # Iterate through stations given above and add to pyrobosim environment.
    room_size = 5
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
        world.add_hallway(room_start=start, room_end=end, name=name, width=1.25, color=[0.2, 0.2, 0.2])

    # Initialise A* Path Planner
    path_planner = AStarPlanner(
        world = world,
        grid_resolution = 1.0,
        grid_inflation_radius = 0.1,
        heuristic = "euclidean",
        diagonal_motion = True,
        compress_path = True
    )
    path_planner.latest_path = None

    # Add trains to network.
    robot = Robot(
        name="Scotrail_170401",
        radius=0.5,
        path_executor=ConstantVelocityExecutor(linear_velocity=5.0),
        path_planner=path_planner,
        color='#1e467d',
        pose=Pose(0, 0, 0, 0, 0, 0)
    )
    world.add_robot(robot, loc="Dundee")

    return world


def create_ros_node():
    """Initializes ROS node"""
    rclpy.init()

    world = create_world()

    node = ExtendedWorldROSWrapper(world)

    node.get_logger().info("World loaded.")

    return node



if __name__ == "__main__":
    node = create_ros_node()

    # Start ROS node in separate thread
    ros_thread = threading.Thread(target=lambda: node.start(wait_for_gui=True))
    ros_thread.start()

    # Start GUI in main thread
    start_gui(node.world)
