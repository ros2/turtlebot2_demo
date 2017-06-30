# Launch localization nodes

On the turtlebot, run the following to launch the ROS 2 AMCL nodes:

```
. ~/ros2_ws/install/setup.bash
launch `ros2 pkg prefix turtlebot2_amcl`/share/turtlebot2_amcl/launch/turtlebot_amcl.py
```

This will start the relevant turtlebot2 nodes, plus AMCL, plus the map server with a demo map.
If you want to use a particular map, specify it with:

```
python3 `ros2 pkg prefix turtlebot2_amcl`/share/turtlebot2_amcl/launch/turtlebot_amcl.py --map ~/maps/mymap.yaml
```

By default, the initial pose will be position (0, 0) in the map with orientation of yaw = 0.
To specify an alternative initial pose, publish a message to the `initialpose` topic, e.g.:

```
ros2 topic pub initialpose geometry_msgs/PoseWithCovarianceStamped "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'   
pose:
  pose:
    position: {x: 5.5, y: 4.5, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  covariance: [0.5, 0.5, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.5, 0.5, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.5]"
```

# Launch visualization with RViz in ROS 1

On a machine that has the ROS 1 <-> ROS 2 bridge installed, you can use the bridge to echo the ROS 2 into ROS 1 topics so that they can be visualized by RViz in ROS 1.

Terminal 1:

```
. /opt/ros/kinetic/setup.bash
roscore
```

Terminal 2:
```
. /opt/ros/kinetic/setup.bash
. ~/ros2_ws/install/setup.bash
ROS_DOMAIN_ID=<turtlebot_ros_domain_id> ros2 run ros1_bridge dynamic_bridge -- --bridge-all-2to1-topics
```

Now RViz will be able to display the localization result.

Terminal 3:
```
. /opt/ros/kinetic/setup.bash
rviz
```

Add TF, PoseArray, LaserArray and Map displays, and you should see the particle cloud move around the map with the robot.

![example localization](https://github.com/ros2/turtlebot2_demo/raw/amcl_readme/turtlebot2_amcl/doc/turtlebot2_amcl_rviz.png "Example of RViz visualization of localization")


## Alternative: using the parameter bridge

If you only want particular topics to be bridged between ROS 1 and ROS 2, you can launch a "parameter bridge" with the provided configuration.

Terminal 2:
```
. /opt/ros/kinetic/setup.bash
. ~/ros2_ws/install/setup.bash
rosparam load `ros2 pkg prefix turtlebot2_amcl`/share/turtlebot2_amcl/config/parameter_bridge.yaml
ROS_DOMAIN_ID=<turtlebot_ros_domain_id> ros2 run ros1_bridge parameter_bridge
```

This will bridge only the relevant topics for using AMCL.
Note that services will not be bridged with this configuration, so you will need to run a map server locally.
