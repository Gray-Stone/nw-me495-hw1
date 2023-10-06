#! /usr/bin/zsh

# Script to run demo with. 
# Usecase: launch the waypoint node and turtle with the launch file first. 
# After recording is started, run this script. 
# Will toggle motion, load waypoints, and toggle motions a bit more. 

# This script should only function as a record of steps during demo. 
# It is not a adaptive nor reusable script!! 

this="$(realpath "$0")"
readonly this="${this}"
here="$(dirname "${this}")"
readonly here="${here}"


GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

info(){
   echo "${RED}[INFO] ${GREEN}$@${NC}"
}

########## Done with shell helper

set -x

info "Launching ros bag capture in background"
(
cd "${here}/../../"
# Remove potential redundent folder
rm -r turtle.bag
ros2 bag record -o turtle.bag /turtle1/cmd_vel
) & 
ROS_BAG_PID=$!

cleanup() {
   info "Killing ros bag capture"
   kill ${ROS_BAG_PID}
}
trap cleanup EXIT

############## 
# Start of demo

sleep 2
info "About to start the ros2 service call sequence"

info "Sending waypoints to /load"
ros2 service call /load turtle_interface/srv/Waypoints \
 '{waypoints: [
    {x: 1.5, y: 1.7 , theta: 0.0},
    {x: 2.1, y: 9.5 , theta: 0.0},
    {x: 7.1, y: 6.0 , theta: 0.0},
    {x: 4.1, y: 2.5 , theta: 0.0},
    {x: 8.1, y: 1.4 , theta: 0.0},
    {x: 4.1, y: 5.2 , theta: 0.0}
   ]}'


sleep 1
info "Starting turtle motion"
ros2 service call /toggle std_srvs/srv/Empty

# turtle running

sleep 7
info "Pausing turtle motion in a move"
ros2 service call /toggle std_srvs/srv/Empty
# turtle Stopped

sleep 2 
info "Resuming turtle motion"
ros2 service call /toggle std_srvs/srv/Empty

# turtle running

info "Wait for rougtly when turtle has gone over one lap"
sleep 12

info "Motion sequence is finished, stopping the control"
ros2 service call /toggle std_srvs/srv/Empty

############
# Clean up after the demo

# Killing the ros bag capture
cleanup

info "Killing the waypoint node through pkill"
pkill waypoint

####################
# Replay 

sleep 1
info "resetting the turtlesim to view replay"
sleep 0.5
ros2 service call /reset std_srvs/srv/Empty

sleep 1
info "Teleporting the turtle to the same location as waypoint 1, to help playback"
sleep 0.2
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute '{x: 1.5 , y: 1.7 , theta: 0.0}'

info "Replay the ros2 bag just captured"
sleep 0.8
ros2 bag play turtle.bag

exit 0
