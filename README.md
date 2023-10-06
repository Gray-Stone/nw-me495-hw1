# ME495 Embedded Systems Homework 1
Author: Leo Chen 
1. Use `ros2 launch turtle_control waypoints.launch.xml` to run the code
2. The `/load` service loads waypoints for the turtle to follow.
    Example: 
    ```bash
    ros2 service call /load turtle_interface/srv/Waypoints \
    '{waypoints: [
        {x: 1.5, y: 1.7 , theta: 0.0},
        {x: 2.1, y: 9.5 , theta: 0.0},
        {x: 7.1, y: 6.0 , theta: 0.0},
        {x: 4.1, y: 2.5 , theta: 0.0},
        {x: 8.1, y: 1.4 , theta: 0.0},
        {x: 4.1, y: 5.2 , theta: 0.0}
    ]}'
    ```
3. The `/toggle` starts and stops the turtle.
    Example: 
    ```bash
    ros2 service call /toggle std_srvs/srv/Empty
    ```
4. Here is a video of the turtle in action.
    
[demo video](https://github.com/Gray-Stone/nw-me495-hw1/assets/7969697/0bb31a22-33d9-4993-abf7-d6040669d852)

---

## ROS-bag Playback Observation 

The trajectory from the rosbag play only includes the velocity, thus it only move turtle relative to current position. The turtle's path will have offset (amount of fist waypoint). In the video, turtle is intentionally teleported to the same starting position before playing the ros-bag. 

Despite matching the starting location. It is obvious the trajectory of the turtle deviates from the original path. By changing the max velocity from waypoint node, the resulting playback error would be reduced. It is likely this deviation is from non-prefect timing of the ros-bag playback.

---

## Helper shell scripts:

`turtle_control` package have a scripts directory (not installed after build). Contains some example shell scripts for calling services. They are used during development of the code. However they are not intended for others to use directly. No compatibility is ensured. They should be treated as example for calling services to control the waypoint node. 