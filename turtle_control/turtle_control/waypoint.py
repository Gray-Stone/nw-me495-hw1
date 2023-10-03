#! /usr/bin/env python3

'''
Control the turtle to move through a bunch of waypoints. 

TODO(LEO) Finish later
'''

from typing import List, Optional
import rclpy
from rclpy.node import Node as RosNode
import std_srvs.srv

from turtle_interface.srv import Waypoints as WaypointsSrv
from geometry_msgs.msg import Pose2D as Pose2DMsg

import enum

from rcl_interfaces.msg import ParameterDescriptor as RosParameterDescriptor
import queue


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
        self.loaded_waypoints: Optional[List[Pose2DMsg]] = None

        # ROS stuff
        self.declare_parameter(
            "frequency", self.DEFAULT_TIMER_FREQUENCY,
            RosParameterDescriptor(
                description="The frequency command is issuing to turtles"))

        self.timer_frequency = self.get_parameter(
            "frequency").get_parameter_value().double_value

        self.timer = self.create_timer(1.0 / self.timer_frequency,
                                       self.timer_callback)

        # Hosted Services
        self.toggle_srv = self.create_service(std_srvs.srv.Empty, "toggle",
                                              self.toggle_srv_callback)
        self.load_srv = self.create_service(WaypointsSrv, "load",
                                            self.load_srv_callback)

    def timer_callback(self) -> None:
        if self.state == self.State.MOVING:
            self.get_logger().debug("Issuing Command")

    def load_srv_callback(
            self, request: WaypointsSrv.Request,
            response: WaypointsSrv.Response) -> WaypointsSrv.Response:
        """Callback to handle "Load" service.

        Argument:
            Request: geo 
            Response:
        Returns:
            WaypointsSrv.Response: the waypoints service response, combined circular length of waypoint route
        """

        # Push the way point request onto queues.
        # print(f"load srv called: {request.waypoints}")
        self.loaded_waypoints = request.waypoints
        print(self.loaded_waypoints)
        # TODO (LEO) Have the turtle draw the way points.

        # TODO (LEO) Fill in later
        response.distance = 0.0
        return response

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
