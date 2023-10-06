#! /usr/bin/env python3
'''
Control the turtle to move through a bunch of waypoints. 

TODO(LEO) Finish later
'''

import enum
import math
from typing import List

import rclpy
import std_srvs.srv
from error_metric_helper import ErrorMeasurer
from geometry_msgs.msg import Pose2D as Pose2DMsg
from geometry_msgs.msg import Twist as TwistMsg
from rcl_interfaces.msg import ParameterDescriptor as RosParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node as RosNode
from turtle_interface.srv import Waypoints as WaypointsSrv
from turtle_interface.msg import ErrorMetric as ErrorMetricMsg
from turtlesim.msg import Pose as TurtlePose
from turtlesim.srv import SetPen, TeleportAbsolute, TeleportRelative


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

    # MAX MIN of angular and linear speeds.
    # We only command positive linear speed.
    LINEAR_VEL_MIN = 0.5
    LINEAR_VEL_MAX = 5.0

    # Angular vel only have max, because it could be positive or negative
    ANGULAR_VEL_MAX = 3.0

    # error * Gain = speed
    ANGULAR_VEL_GAIN = 4.0
    LINEAR_VEL_GAIN = 5.0

    # Start linear motion when speed is at this threshold.
    # 90 deg is 1.5 ish rad
    PURE_ROTATION_THRESHOLD = 3

    DEFAULT_TIMER_FREQUENCY: float = 100.0  # 100hz
    DEFAULT_ON_WAYPOINT_DISTANCE_TOLERANCE: float = 0.1

    class State(enum.Enum):
        '''Internal state of the waypoint node
        '''
        STOPPED = 0
        MOVING = 1

    def __init__(self) -> None:
        super().__init__("waypoint")

        # Integral variables
        self._state = self.State.STOPPED
        self._loaded_waypoints: List[Pose2DMsg] = []
        # Start off with empty turtle pose
        self._turtle_pose: TurtlePose = TurtlePose()
        # This is for timer to remember it's targe. Should be reset on load.
        self._current_waypoint_index: int = 0

        self._error_measure = ErrorMeasurer(0.0)

        # ROS stuff
        self.declare_parameter(
            "frequency", self.DEFAULT_TIMER_FREQUENCY,
            RosParameterDescriptor(description="The frequency command is issuing to turtles"))
        self.declare_parameter(
            "tolerance", self.DEFAULT_ON_WAYPOINT_DISTANCE_TOLERANCE,
            RosParameterDescriptor(description="The tolerance (distance) to consider turtle on the waypoint"))

        self._timer_frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self._on_waypoint_distance_tolerance = self.get_parameter(
            "tolerance").get_parameter_value().double_value

        self._turtle_control_timer = self.create_timer(1.0 / self._timer_frequency, self.timer_callback)

        # Hosted Services
        self._toggle_srv = self.create_service(std_srvs.srv.Empty, "toggle", self.toggle_srv_callback)
        self._load_srv = self.create_service(WaypointsSrv, "load", self.load_srv_callback)

        # Client for moving turtle

        # Separate callback group for client to call other nodes.
        self._drawing_callback_group = MutuallyExclusiveCallbackGroup()
        self._pose_ctl_callback_group = MutuallyExclusiveCallbackGroup()

        # TODO (LEO) The turtle name might need fixing
        self._reset_client = self.create_client(std_srvs.srv.Empty,
                                                '/reset',
                                                callback_group=self._drawing_callback_group)
        self._reset_client.wait_for_service()

        self._teleport_abs_client = self.create_client(TeleportAbsolute,
                                                       "/turtle1/teleport_absolute",
                                                       callback_group=self._drawing_callback_group)
        self._teleport_abs_client.wait_for_service()

        self._teleport_rel_client = self.create_client(TeleportRelative,
                                                       "/turtle1/teleport_relative",
                                                       callback_group=self._drawing_callback_group)
        self._teleport_rel_client.wait_for_service()

        self._pen_client = self.create_client(SetPen,
                                              "/turtle1/set_pen",
                                              callback_group=self._drawing_callback_group)
        self._pen_client.wait_for_service()

        # Subscriber and publisher
        self._pos_subs = self.create_subscription(TurtlePose,
                                                  '/turtle1/pose',
                                                  self.turtle_pose_recv_callback,
                                                  10,
                                                  callback_group=self._pose_ctl_callback_group)

        self._cmd_publisher = self.create_publisher(TwistMsg, '/turtle1/cmd_vel', 10)

        self._error_metric_publisher = self.create_publisher(ErrorMetricMsg, 'loop_metrics', 10)

    def timer_callback(self) -> None:
        if self._state == self.State.MOVING:
            self.get_logger().debug("Issuing Command")

            if not self._loaded_waypoints:
                self.get_logger().error("No waypoints loaded. Load them with the 'load' service.")
            else:
                # Calculate target velocity to move towards target
                current_waypoint = self._loaded_waypoints[self._current_waypoint_index]

                # Note the order of which subtract which really matter here. It could result in flipped sign
                dx = current_waypoint.x - self._turtle_pose.x
                dy = current_waypoint.y - self._turtle_pose.y
                pos_error = math.sqrt((dx)**2 + (dy)**2)

                target_heading = math.atan2(dy, dx)
                heading_error = map_into_180(target_heading - self._turtle_pose.theta)
                # Advance the waypoint when we are close enough to current one
                if pos_error < self._on_waypoint_distance_tolerance:
                    self.get_logger().info(f"Reached waypoint at {current_waypoint.x , current_waypoint.y}")
                    self._current_waypoint_index = self._current_waypoint_index + 1
                    if self._current_waypoint_index >= len(self._loaded_waypoints):
                        self._current_waypoint_index = 0
                    elif self._current_waypoint_index == 1:
                        print("About to log another lap")
                        self._error_measure.finished_another_loop()
                        self.get_logger().info(
                            f"Finished another loop, looped {self._error_measure.completed_loops} times")
                        self.get_logger().info(f"Error Metrics: {self._error_measure}")

                        self._error_metric_publisher.publish(
                            self._error_measure.generate_error_metric_message())

                cmd = self.calculate_command(heading_error, pos_error)

                self.get_logger().debug(f"""
                Heading to target index {self._current_waypoint_index}
                Heading: target {target_heading}; turtle {self._turtle_pose.theta}; error {heading_error}; angle-z-speed {cmd.angular.z}
                Pos error : {pos_error}; linear-x-speed {cmd.linear.x} 
                                        """)
                self._cmd_publisher.publish(cmd)
        elif self._state == self.State.STOPPED:
            pass
        else:
            # There should not be accidental fall through
            raise ValueError(f"State enum of {self._state} Is not handled ! ")

    def calculate_command(self, heading_error: float, pos_error: float) -> TwistMsg:

        return_twist = TwistMsg()
        angular_var = heading_error * self.ANGULAR_VEL_GAIN
        return_twist.angular.z = max(-self.ANGULAR_VEL_MAX, min(angular_var, self.ANGULAR_VEL_MAX))

        return_twist.linear.x = 0.0
        return_twist.linear.y = 0.0

        if abs(heading_error) < self.PURE_ROTATION_THRESHOLD:
            # The heading error is not too bad, we are allowed to start linear motion
            linear_speed = pos_error * self.LINEAR_VEL_GAIN
            return_twist.linear.x = max(self.LINEAR_VEL_MIN, min(linear_speed, self.LINEAR_VEL_MAX))

        # Re-center heading-enter into -180 to 180
        return return_twist

    def turtle_pose_recv_callback(self, msg: TurtlePose) -> None:
        '''
        Store the received turtle position message.
        '''
        # Measure how much turtle has moved forward
        dx = msg.x - self._turtle_pose.x
        dy = msg.y - self._turtle_pose.y
        distance = math.sqrt(dx**2 + dy**2)

        self._error_measure.add_traveled_distance(distance)

        self._turtle_pose = msg

    async def load_srv_callback(self, request: WaypointsSrv.Request,
                                response: WaypointsSrv.Response) -> WaypointsSrv.Response:
        """Callback to handle "Load" service.

        Argument:
            Request: geo 
            Response:
        Returns:
            WaypointsSrv.Response: the waypoints service response, 
            combined circular length of waypoint route
        """

        self.get_logger().info(f"Load service is called with {len(request.waypoints)} waypoints")

        # Make sure the motion control is stopped.
        if self._state == self.State.MOVING:
            self.get_logger().warning(
                "Received /Load during motion. Stopping robot to reset and load new waypoints")
            self.set_stopped()
        # Reset everything.
        await self._reset_client.call_async(std_srvs.srv.Empty_Request())
        self._loaded_waypoints = []

        # We don't do anything with less then 2 waypoints
        if len(request.waypoints) > 1:
            # Draw all the way points
            for waypoint in request.waypoints:
                await self.turtle_draw_waypoint(waypoint)

            # Accumulate distance. This is a bit tricker because we need a loop of distance
            response.distance = 0.0
            for i in range(len(request.waypoints)):
                dx = request.waypoints[i].x - request.waypoints[i - 1].x
                dy = request.waypoints[i].y - request.waypoints[i - 1].y
                response.distance += math.sqrt(dx**2 + dy**2)

            # Place turtle at first waypoint
            # Push the way point request onto queues.
            self._loaded_waypoints = request.waypoints
            self._current_waypoint_index = 1
            start_point = request.waypoints[0]
            await self._teleport_abs_client.call_async(
                TeleportAbsolute.Request(x=start_point.x, y=start_point.y, theta=start_point.theta))
            self.get_logger().info("Turtle is ready at the start of waypoint")

        self._error_measure = ErrorMeasurer(response.distance)
        return response

    def toggle_srv_callback(self, _: std_srvs.srv.Empty.Request,
                            response: std_srvs.srv.Empty.Response) -> std_srvs.srv.Empty.Response:
        # TODO(LEO) remove later
        print("toggle service called")

        if self._state == self.State.MOVING:
            self.set_stopped()
            self.get_logger().info("Stopping")
        else:
            self._state = self.State.MOVING
            self.get_logger().debug("Switching into moving state ")
        return response

    async def turtle_draw_waypoint(self, waypoint: Pose2DMsg):
        '''
        Use ros messages to control turtle to draw X for a specific waypoint
        '''
        # iterate through waypoints and draw them
        await self.turn_pen_on(False)

        for i in range(2):
            # Centered at waypoint
            ang = math.pi / 4 + i * math.pi / 2

            await self._teleport_abs_client.call_async(
                TeleportAbsolute.Request(x=waypoint.x, y=waypoint.y, theta=waypoint.theta))
            # Draw one line of the X
            await self.turn_pen_on(True)
            await self._teleport_rel_client.call_async(TeleportRelative.Request(linear=0.5, angular=ang))
            await self._teleport_rel_client.call_async(
                TeleportRelative.Request(linear=0.5 * 2, angular=math.pi))
            await self.turn_pen_on(False)

    async def turn_pen_on(self, on: bool = True) -> None:
        '''
        Use set-pen service to turn pen on or off.
        Args:
            ON: bool -> true to turn pen on, false to turn pen off
        '''

        pen_req = SetPen.Request(r=70, g=70, b=30, width=2, off=not on)
        ret = await self._pen_client.call_async(pen_req)
        if ret is None:
            raise RuntimeError(f"Failed to get pen service return. Got {ret}")

    def set_stopped(self) -> None:
        '''
        Change to stopped state as well as stop robot moving.
        '''
        self._state = self.State.STOPPED

        # Depend on default twist msg being all zeros
        self._cmd_publisher.publish(TwistMsg())


def map_into_180(rad: float) -> float:
    '''
    Angle + pi*2 is the same angel. Map any angle back into -180 180 deg  
    '''

    # With only the ratio calculation, everything is mapped within +- 2pi
    # Give it an extra nudge of pi so the reduction ratio kicks in half pi earlier.
    # As long as we only subtract multiples of pi*2, it's find for any random nudge when computing the ratio
    ratio = 0
    if rad > math.pi:
        ratio = (rad + math.pi) / (math.pi * 2)

    elif rad < math.pi:
        ratio = (rad - math.pi) / (math.pi * 2)

    reduced = rad - int(ratio) * (math.pi * 2)
    return reduced


if __name__ == '__main__':
    main()

    # # Manual testing the map_angel_back
    # for i in range(100000):
    #     input = i / 1000.0
    #     output = map_angle_back(input)
    #     if not math.isclose(math.cos(input), math.cos(output)):
    #         print(
    #             f"Answer diff with cos! in {input} out {output}. cos output {math.cos(input), math.cos(output)}"
    #         )
    #     if not math.isclose(math.sin(input), math.sin(output)):
    #         print(
    #             f"Answer diff with sin! in {input} out {output} , sin output {math.sin(input), math.sin(output)}"
    #         )
    #     if output > math.pi or output < -math.pi:
    #         print(f"Output out of range {output}")
