import rclpy
from rclpy.node import Node


from pyrobosim_msgs.action import FollowPath, PlanPath
from pyrobosim_msgs import msg


class RailTrafficController(Node):
    def __init__(self):
        super().__init__("RailTrafficController")


def main():
    print('Hi from trafficsim.')


if __name__ == '__main__':
    main()
