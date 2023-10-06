#! /usr/bin/zsh

# A quick shell script on early stage of development to test waypoint node with some service calls.
# Intended for manual debuging only

# set -euvx

this="$(realpath "$0")"
readonly this="${this}"
here="$(dirname "${this}")"
readonly here="${here}"

# We don't want to throw ros node in backgroun, Because it always cause issue.
# So the service call that's happening after node being created, will happen first.
(
    set -euvx

    sleep 0.5
    timeout -s TERM 1m ros2 service call /toggle std_srvs/srv/Empty
    sleep 1

    timeout -s TERM 1m ros2 service call /load turtle_interface/srv/Waypoints \
        '{waypoints: [{x: 2.0 , y: 1.0 , theta: 0.0}
        , {x: 3.0 , y: 9.0 , theta: 0.8}
        , {x: 10.0 , y: 6.0 , theta: 2.8}]}'

    sleep 0.5
    timeout -s TERM 1m ros2 service call /toggle std_srvs/srv/Empty

    sleep 9
    echo "ROS terminal call"
    timeout -s TERM 1m ros2 service call /load turtle_interface/srv/Waypoints \
        '{waypoints: [{x: 2.0 , y: 1.0 , theta: 0.0}
 , {x: 3.0 , y: 9.0 , theta: 0.8}
 , {x: 10.0 , y: 6.0 , theta: 2.8}]}'
    timeout -s TERM 1m ros2 service call /toggle std_srvs/srv/Empty

    while true; do
        sleep 1
        timeout -s TERM 1m ros2 service call /toggle std_srvs/srv/Empty
        sleep 1
        timeout -s TERM 1m ros2 service call /toggle std_srvs/srv/Empty
    done
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

ros2 run turtle_control waypoint.py
exit 0
