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
import rclpy
from rclpy.node import Node

# Import Message, Server, and Action Interfaces
from rclpy.action import ActionServer, ActionClient
from pyrobosim_msgs.srv import RequestWorldState
from pyrobosim_msgs.action import FollowPath, PlanPath
from trafficsim_interfaces.action import ExecuteTrainRoute
from std_srvs.srv import Trigger


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

class TrainController(Node):

    # ------------------------------------------------------------------------------------
    # 
    # Class Constructor
    #
    # ------------------------------------------------------------------------------------
    def __init__(self):
        super().__init__("Train_Controller")

        # Create Action Server to handle train routing.
        self.train_routing_action = ActionServer(
            self,
            ExecuteTrainRoute,
            'execute_train_route',
            self.train_routing_callback
        )

        # Create Service to get world state
        self.world_state_service = self.create_client(
            RequestWorldState,
            'request_world_state'
        )
    
    # ------------------------------------------------------------------------------------
    # 
    # NAME:         _get_world_state()
    # DESCRIPTION:  Calls the /request_world_state service to retrieve information about
    #               current robots (trains), hallways (lines), and rooms (stations).
    # PARAMETERS:   goal_handle - the Action Goal object passed in from the Action Server callback.
    # RETURNS:      future - an object representing the response of the service call, if successful.
    #               -1 - an int if the request fails due to server being unavailable.
    #
    # ------------------------------------------------------------------------------------
    def _get_world_state(self, goal_handle):
        if not self.world_state_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service \'/request_world_state\' not available.')
            return -1
        world_state_request = RequestWorldState.Request()
        world_state_request.robot = goal_handle.request.train_id
        future = self.world_state_service.call_async(world_state_request)
        return future
    
    async def _send_planning_goal(self, goal_handle, target):
        self.planning_client = ActionClient(
            self,
            PlanPath,
            f"{goal_handle.request.train_id}/plan_path"
        )
        if not self.planning_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'{goal_handle.request.train_id} - Cannot initialise planning client - server unavailable. Aborting...')
            goal_handle.abort()
            return ExecuteTrainRoute.Result(success=False)
        planning_goal = PlanPath.Goal()
        planning_goal.target_location = target

        planning_goal_handle = await self.planning_client.send_goal_async(planning_goal)

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

        self.planning_client = None

        return planning_result
    
    async def _send_follow_goal(self, goal_handle, path):
        self.follow_client = ActionClient(
            self,
            FollowPath,
            f"{goal_handle.request.train_id}/follow_path"
        )

        if not self.follow_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'{goal_handle.request.train_id} - Cannot initialise follow client - server unavailable. Aborting...')
            goal_handle.abort()
            return ExecuteTrainRoute.Result(success=False)

        follow_goal = FollowPath.Goal()
        follow_goal.path = path

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

        self.follow_client = None
        
        return follow_result

    # Action Server for train routing. REF: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html
    # Ref 2 - https://www.velotio.com/engineering-blog/async-features-in-python - needed to call other async functions (services)
    async def train_routing_callback(self, goal_handle):
        # Use the /request_world_state service to get information about current robots.
        world_state = await self._get_world_state(goal_handle)
        if world_state == -1:
            goal_handle.abort()
            return ExecuteTrainRoute.Result(success=False)

        # Use response from /request_world_state to check if robot name passed in as parameter is valid.
        # If not, then abort this action.
        if goal_handle.request.train_id not in [robot.name for robot in world_state.state.robots]:
            self.get_logger().error('Train ID not in Pyrobosim environment, aborting routing attempt.')
            goal_handle.abort()
            return ExecuteTrainRoute.Result(success=False)

        # Get current robot state.
        current_robot_state = [robot for robot in world_state.state.robots if robot.name == goal_handle.request.train_id][0]

        # Use response from /request_world_state to check if all stations passed in as parameters exist
        # within the environment. If not, then abort.
        nonexistent_stations = set(goal_handle.request.stops).difference(
            set([hallway.room_start for hallway in world_state.state.hallways] +
                [hallway.room_end for hallway in world_state.state.hallways]
            )
        )
        if len(nonexistent_stations) > 0:
            self.get_logger().error('One or more Station IDs in "stops" parameter not in Pyrobosim environment, aborting routing attempt.')
            goal_handle.abort()
            return ExecuteTrainRoute.Result(success=False)

        # # Setup train planning route.
        # train_route = []
        # for stop in goal_handle.request.stops:
        #     train_route.append(stop)
        #     train_route.append(f"{stop}_Platform")
        # train_route.append(goal_handle.request.destination)
        # train_route.append(f"{goal_handle.request.destination}_Platform")

        # Train name is correct, we now attempt to create the train route specified in the parameters.
        self.get_logger().info(f'Routing Train ID: {goal_handle.request.train_id} to destination: {goal_handle.request.destination}...')
        self.get_logger().info(f'This train will call at: {', '.join(goal_handle.request.stops[:-1])}')

        # for stop in train_route:
        #     planning_result = await self._send_planning_goal(goal_handle, stop)
        #     follow_result = await self._send_follow_goal(goal_handle, planning_result.result.path)
        #     self.get_logger().info(f"{goal_handle.request.train_id} - Stopped at: {str(stop)}.")

        for i in range(0, len(goal_handle.request.stops)+1):
            planning_result = await self._send_planning_goal(goal_handle, goal_handle.request.stops[i] if i < len(goal_handle.request.stops) else goal_handle.request.destination)

            self.get_logger().info(f'{goal_handle.request.train_id} - Route planned successfully.')

            follow_result = await self._send_follow_goal(goal_handle, planning_result.result.path)

            self.get_logger().info(f'{goal_handle.request.train_id} - Path segment traversed successfully.')

            if i < len(goal_handle.request.stops):
                self.get_logger().info(f'{goal_handle.request.train_id} - Stopped at: {goal_handle.request.stops[i]}. Next stop: {goal_handle.request.destination if i == len(goal_handle.request.stops)-1 else goal_handle.request.stops[i+1]}.')
            else:
                self.get_logger().info(f'{goal_handle.request.train_id} - Stopped at: {goal_handle.request.destination}, where this train will terminate.')
                self.get_logger().info(f'{goal_handle.request.train_id} - Train has reached final destination - path complete.')
            
            planning_result = None
            follow_result = None

            time.sleep(1)

        # # TODO: Proof-of-Concept for Action Feedback - Implement this each time the train stops at a designated stop in the list!
        # fb = ExecuteTrainRoute.Feedback()
        # for i in range(0,5):
        #     fb.visited.append('stop' + str(i))
        #     goal_handle.publish_feedback(fb)        # Publishes to a topic /execute_train_route/_action/feedback
        #     time.sleep(1)

        goal_handle.succeed()

        result = ExecuteTrainRoute.Result(success=True)
        return result


# ----------------------------------------------------------------------------------------
# 
# NAME:         main()
# DESCRIPTION:  The main entry point for this module.
# PARAMETERS:   none
# RETURNS:      none
#
# REFERENCES:   - https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#id1
#
# ----------------------------------------------------------------------------------------

def main():
    rclpy.init()
    node = TrainController()
    node.get_logger().info("PyRoboSim may take a few minutes to build the simulation environment, please wait...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
