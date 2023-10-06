#! /usr/bin/zsh

# set -euvx

# Script to run demo with. 
# Usecase: launch the waypoint node and turtle with the launch file first. 
# After recording is started, run this script. 
# Will toggle motion, load waypoints, and toggle motions a bit more. 

this="$(realpath "$0")"
readonly this="${this}"
here="$(dirname "${this}")"
readonly here="${here}"

# We don't want to throw ros node in backgroun, Because it always cause issue.
# So the service call that's happening after node being created, will happen first.

set -euvx

(
cd "${here}/../../capture"
ros2 bag record /turtle1/cmd_vel

)

exit 0

sleep 0.5
ros2 service call /toggle std_srvs/srv/Empty
# turtle running
sleep 0.2
ros2 service call /toggle std_srvs/srv/Empty
# turtle stopped
sleep 0.2

ros2 service call /load turtle_interface/srv/Waypoints \
 '{waypoints: [
    {x: 1.5, y: 1.7 , theta: 0.0},
    {x: 2.1, y: 9.5 , theta: 0.1},
    {x: 7.1, y: 6.0 , theta: 0.2},
    {x: 4.1, y: 2.5 , theta: 0.3},
    {x: 8.1, y: 1.4 , theta: 0.4},
    {x: 4.1, y: 5.2 , theta: 0.5}
   ]}'

sleep 2
ros2 service call /toggle std_srvs/srv/Empty
# turtle running

sleep 3
ros2 service call /toggle std_srvs/srv/Empty
# turtle Stopped

sleep 0.7
ros2 service call /toggle std_srvs/srv/Empty
# turtle running


exit 0
