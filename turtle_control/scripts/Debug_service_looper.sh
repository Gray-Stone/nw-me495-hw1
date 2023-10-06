#! /usr/bin/zsh

# set -euvx

this="$(realpath "$0")"
readonly this="${this}"
here="$(dirname "${this}")"
readonly here="${here}"

# We don't want to throw ros node in backgroun, Because it always cause issue.
# So the service call that's happening after node being created, will happen first.

set -euvx

sleep 0.5
ros2 service call /toggle std_srvs/srv/Empty
sleep 1

ros2 service call /load turtle_interface/srv/Waypoints \
    '{waypoints: [{x: 2.0 , y: 1.0 , theta: 0.0}
 , {x: 3.0 , y: 9.0 , theta: 0.8}
 , {x: 10.0 , y: 6.0 , theta: 2.8}]}'

sleep 0.5
ros2 service call /toggle std_srvs/srv/Empty

sleep 9
echo "ROS terminal call"
ros2 service call /load turtle_interface/srv/Waypoints \
    '{waypoints: [{x: 2.0 , y: 1.0 , theta: 0.0}
 , {x: 3.0 , y: 9.0 , theta: 0.8}
 , {x: 10.0 , y: 6.0 , theta: 2.8}]}'
ros2 service call /toggle std_srvs/srv/Empty

while true; do
    sleep 1
    ros2 service call /toggle std_srvs/srv/Empty
    sleep 1
    ros2 service call /toggle std_srvs/srv/Empty
done



exit 0
