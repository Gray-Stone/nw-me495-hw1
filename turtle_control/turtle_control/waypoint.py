#! /usr/bin/env python3
'''
Control the turtle to move through a bunch of waypoints. 

TODO(LEO) Finish later
'''

from typing import List, Optional
import rclpy
from rclpy.node import Node as RosNode
import std_srvs.srv

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from turtle_interface.srv import Waypoints as WaypointsSrv
from geometry_msgs.msg import Pose2D as Pose2DMsg
from turtlesim.srv import TeleportAbsolute, TeleportRelative, SetPen

import enum

import math

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
        self.loaded_waypoints: List[Pose2DMsg] = []

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

        # Client for movnig turtle

        # Seperate callback group for client to call other nodes.
        self.drawing_callback_group = MutuallyExclusiveCallbackGroup()

        # TODO (LEO) The turtle name might need fixing
        self.reset_client = self.create_client(
            std_srvs.srv.Empty,
            '/reset',
            callback_group=self.drawing_callback_group)
        self.reset_client.wait_for_service()

        self.teleport_abs_client = self.create_client(
            TeleportAbsolute,
            "/turtle1/teleport_absolute",
            callback_group=self.drawing_callback_group)
        self.teleport_abs_client.wait_for_service()

        self.teleport_rel_client = self.create_client(
            TeleportRelative,
            "/turtle1/teleport_relative",
            callback_group=self.drawing_callback_group)
        self.teleport_rel_client.wait_for_service()

        self.pen_client = self.create_client(
            SetPen,
            "/turtle1/set_pen",
            callback_group=self.drawing_callback_group)
        self.pen_client.wait_for_service()

    def timer_callback(self) -> None:
        if self.state == self.State.MOVING:
            self.get_logger().debug("Issuing Command")

    async def load_srv_callback(
            self, request: WaypointsSrv.Request,
            response: WaypointsSrv.Response) -> WaypointsSrv.Response:
        """Callback to handle "Load" service.

        Argument:
            Request: geo 
            Response:
        Returns:
            WaypointsSrv.Response: the waypoints service response, combined circular length of waypoint route
        """

        self.get_logger().info(
            f"Load service is called with {len(request.waypoints)} waypoitns")

        # Reset everything.
        await self.reset_client.call_async(std_srvs.srv.Empty_Request())

        # Draw all the way points
        for waypoint in request.waypoints:
            await self.turtle_draw_waypoint(waypoint)

        # Accumulate distance. This is a bit tricker because we need a loop of distance
        response.distance = 0.0
        # We have no distance to calculate if it's less then 2 points
        if len(request.waypoints) > 1:
            for i in range(len(request.waypoints)):
                dx = request.waypoints[i].x - request.waypoints[i - 1].x
                dy = request.waypoints[i].y - request.waypoints[i - 1].y
                response.distance += math.sqrt(dx**2 + dy**2)

        # Place turtle at first waypoint
        # It is possible we got empty waypoint, skip.
        if request.waypoints:
            # Push the way point request onto queues.
            self.loaded_waypoints = request.waypoints

            start_point = request.waypoints[0]
            await self.teleport_abs_client.call_async(
                TeleportAbsolute.Request(x=start_point.x,
                                         y=start_point.y,
                                         theta=start_point.theta))
            self.get_logger().info("Turtle is ready at the start of waypoint")

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

    async def turtle_draw_waypoint(self, waypoint: Pose2DMsg):
        '''
        Use ros messages to control turtlism to draw X for a specific waypoint
        '''
        # iterate through waypoitns and draw them
        await self.turn_pen_on(False)

        for i in range(2):
            # Centered at waypoint
            ang = math.pi / 4 + i * math.pi / 2

            await self.teleport_abs_client.call_async(
                TeleportAbsolute.Request(x=waypoint.x,
                                         y=waypoint.y,
                                         theta=waypoint.theta))
            # Draw one line of the X
            await self.turn_pen_on(True)
            await self.teleport_rel_client.call_async(
                TeleportRelative.Request(linear=0.5, angular=ang))
            await self.teleport_rel_client.call_async(
                TeleportRelative.Request(linear=0.5 * 2, angular=math.pi))
            await self.turn_pen_on(False)

    async def turn_pen_on(self, on: bool = True) -> None:
        '''
        Use set-pen service to turn pen on or off.
        Args:
            ON: bool -> true to turn pen on, false to turn pen off
        '''

        pen_req = SetPen.Request(r=70, g=70, b=30, width=2, off=not on)
        ret = await self.pen_client.call_async(pen_req)
        if ret is None:
            raise RuntimeError(f"Failed to get pen service return. Got {ret}")


if __name__ == '__main__':
    main()
