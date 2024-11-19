import rclpy
from rclpy.node import Node


from pyrobosim_msgs.action import FollowPath, PlanPath
from pyrobosim_msgs import msg


class RailTrafficController(Node):

    # TODO: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html
    # Refer to Action Client documentation - implement in here to allow us to programmatically control the robot's
    # routing on the Pyrobosim node.

    def __init__(self):
        super().__init__("RailTrafficController")


def main():
    print('Hi from trafficsim.')


if __name__ == '__main__':
    main()
