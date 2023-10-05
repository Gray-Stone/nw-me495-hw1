#! /usr/bin/zsh

# set -euvx

this="$(realpath "$0")"
readonly this="${this}"
here="$(dirname "${this}")"
readonly here="${here}"

# We don't want to throw ros node in backgroun, Because it always cause issue. 
# So the service call that's happening after node being created, will happen first.
( 
sleep 1 ;
timeout -s TERM 1m ros2 service call /load turtle_interface/srv/Waypoints \
'{waypoints: [{x: 2.0 , y: 1.0 , theta: 0.0}
 , {x: 3.0 , y: 9.0 , theta: 0.8}
 , {x: 7.0 , y: 3.0 , theta: 0.8} ,{x: 5.0 , y: 6.0 , theta: 0.0}
 , {x: 10.0 , y: 6.0 , theta: 2.8}]}'
) &
SRV_CALL_PID=$!

# (
# ros2 run turtlesim  turtlesim_node
# ) &
# TURTLE_PID=$!

trap -l


cleanup() {
echo "Clean up: killing other bg processes"

kill -TERM ${SRV_CALL_PID}
kill -TERM ${TURTLE_PID}

echo "Everythign should be killed now"

ps ${SRV_CALL_PID}
ps ${TURTLE_PID}
}

trap cleanup EXIT SIGINT
# trap cleanup SIGINT

"${here}/../turtle_control/waypoint.py" 
# "${here}/../turtle_control/waypoint.py" --ros-args --log-level debug
exit 0