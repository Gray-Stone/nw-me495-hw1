'''
Control the turtle to move through a bunch of waypoints. 

TODO(LEO) Finish later
'''

from typing import List, Optional
import rclpy
from rclpy.node import Node as RosNode
import std_srvs.srv
import enum

from rcl_interfaces.msg import ParameterDescriptor as RosParameterDescriptor


def main(args=None):
    '''
    The main entry point. This combines entry method for both ros2 run through
    setup.py as well as direct file executaion 
    '''
    # This allows verifying the entry point
    print(f'Starting at main of turtle control, given args: {args}')
    rclpy.init(args=args)
    waypoint_node = WaypointNode()
    rclpy.spin(waypoint_node)
    rclpy.shutdown()


class WaypointNode(RosNode):
    '''
    TODO(LEO)
    '''

    DEFAULT_TIMER_FREQUENCY: float = 100.0  # 100hz

    class State(enum.Enum):
        '''Internal state of the waypoint node
        '''
        STOPPED = 0
        MOVING = 1

    def __init__(self) -> None:
        super().__init__("waypoint")

        # Interal variables
        self.state = self.State.STOPPED

        # ROS stuff
        self.declare_parameter(
            "frequency", self.DEFAULT_TIMER_FREQUENCY,
            RosParameterDescriptor(
                description="The frequency command is issuing to turtles"))

        self.timer_frequency = self.get_parameter(
            "frequency").get_parameter_value().double_value

        self.timer = self.create_timer(1.0 / self.timer_frequency,
                                       self.timer_callback)
        self.toggle_srv = self.create_service(std_srvs.srv.Empty, "toggle",
                                              self.toggle_srv_callback)

    def timer_callback(self) -> None:
        if self.state == self.State.MOVING:
            self.get_logger().debug("Issuing Command")

    def toggle_srv_callback(
            self, _: std_srvs.srv.Empty.Request,
            response: std_srvs.srv.Empty.Response
    ) -> std_srvs.srv.Empty.Response:
        # TODO(LEO) remove later
        print("toggel service called")

        if self.state == self.State.MOVING:
            self.state = self.State.STOPPED
            self.get_logger().info("Stopping")
        else:
            self.state = self.State.MOVING
            self.get_logger().debug("Switching into moving state ")
        return response


if __name__ == '__main__':
    main()
