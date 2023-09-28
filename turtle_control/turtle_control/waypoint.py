'''
Control the turtle to move through a bunch of waypoints. 

TODO(LEO) Finish later
'''

from typing import List, Optional
import rclpy
from rclpy.node import Node as RosNode


def main(args=None):
    '''
    The main entry point. This joint entry method for both ros2 run through setup.py as well as direct file executaion 
    '''
    # This allows verifying the entry point
    print(f'Starting at main of turtle control, given args: {args}')
    rclpy.init(args=args)
    waypoint_node = WaypointNode()

    rclpy.spin(waypoint_node)
    rclpy.shutdown()

    


class WaypointNode(RosNode):

    def __init__(self) -> None:
        super().__init__("waypoint")
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self) -> None:
        self.get_logger().info("Timer_callback gets used")


if __name__ == '__main__':
    main()
