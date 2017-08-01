# Launch localization nodes

On the turtlebot, run the following to launch the ROS 2 AMCL nodes:

```
. /opt/ros/kinetic/setup.bash
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

You should see the following console output that says that the pose estimate is being updated as the robot moves around:

```
$ launch `ros2 pkg prefix turtlebot2_amcl`/share/turtlebot2_amcl/launch/turtlebot_amcl.py
(kobuki_node) pid 16833: ['/home/ubuntu/tb2_demo_ws/install_isolated/turtlebot2_drivers/lib/turtlebot2_drivers/kobuki_node'] (stderr > stdout, all > console)
(astra_camera_node) pid 16834: ['/home/ubuntu/tb2_demo_ws/install_isolated/astra_camera/lib/astra_camera/astra_camera_node', '-dw', '320', '-dh', '240', '-C', '-I'] (stderr > stdout, all > console)
(depthimage_to_laserscan_node) pid 16835: ['/home/ubuntu/tb2_demo_ws/install_isolated/depthimage_to_laserscan/lib/depthimage_to_laserscan/depthimage_to_laserscan_node'] (stderr > stdout, all > console)
(static_tf_pub_base_rgb) pid 16836: ['/home/ubuntu/tb2_demo_ws/install_isolated/tf2_ros/lib/tf2_ros/static_transform_publisher', '-0.087', '-0.0125', '0.287', '0', '0', '0', '1', 'base_link', 'camera_rgb_frame'] (stderr > stdout, all > console)
(static_tf_pub_rgb_depth) pid 16839: ['/home/ubuntu/tb2_demo_ws/install_isolated/tf2_ros/lib/tf2_ros/static_transform_publisher', '0', '0.0250', '0', '0', '0', '0', '1', 'camera_rgb_frame', 'camera_depth_frame'] (stderr > stdout, all > console)
(joy_node) pid 16850: ['/home/ubuntu/tb2_demo_ws/install_isolated/joy/lib/joy/joy_node'] (stderr > stdout, all > console)
(teleop_node) pid 16853: ['/home/ubuntu/tb2_demo_ws/install_isolated/teleop_twist_joy/lib/teleop_twist_joy/teleop_node'] (stderr > stdout, all > console)
(map_server) pid 16869: ['/home/ubuntu/tb2_demo_ws/install_isolated/map_server/lib/map_server/map_server', '/home/ubuntu/tb2_demo_ws/install_isolated/turtlebot2_amcl/share/turtlebot2_amcl/examples/osrf_map.yaml'] (stderr > stdout, all > console)
(amcl) pid 16883: ['/home/ubuntu/tb2_demo_ws/install_isolated/amcl/lib/amcl/amcl', '--use-map-topic'] (stderr > stdout, all > console)
[astra_camera_node] Device "2bc5/0401@2/8" found.
[astra_camera_node] Using depth frame id openni_depth_optical_frame
[astra_camera_node] Using color frame id openni_color_optical_frame
[amcl] [INFO] []: Received a 4000 X 4000 map @ 0.050 m/pix (handleMapMessage() at /home/ubuntu/tb2_demo_ws/src/ros2/navigation/amcl/src/amcl_node.cpp:914)
[amcl] [INFO] []: Initializing likelihood field model; this can take some time on large maps... (handleMapMessage() at /home/ubuntu/tb2_demo_ws/src/ros2/navigation/amcl/src/amcl_node.cpp:980)
[amcl] [INFO] []: Done initializing likelihood field model. (handleMapMessage() at /home/ubuntu/tb2_demo_ws/src/ros2/navigation/amcl/src/amcl_node.cpp:983)
[amcl] [DEBUG] []: Setting up laser 0 (frame_id=camera_depth_frame) (laserReceived() at /home/ubuntu/tb2_demo_ws/src/ros2/navigation/amcl/src/amcl_node.cpp:1179)
[amcl] [DEBUG] []: Received laser's pose wrt robot: -0.087 0.013 0.000 (laserReceived() at /home/ubuntu/tb2_demo_ws/src/ros2/navigation/amcl/src/amcl_node.cpp:1209)
[amcl] [DEBUG] []: Laser 0 angles in base frame: min: -0.510 inc: 0.003 (laserReceived() at /home/ubuntu/tb2_demo_ws/src/ros2/navigation/amcl/src/amcl_node.cpp:1329)
[amcl] [DEBUG] []: Num samples: 5000 (laserReceived() at /home/ubuntu/tb2_demo_ws/src/ros2/navigation/amcl/src/amcl_node.cpp:1371)
[amcl] [DEBUG] []: Max weight pose: 0.013 -0.002 0.000 (laserReceived() at /home/ubuntu/tb2_demo_ws/src/ros2/navigation/amcl/src/amcl_node.cpp:1425)
[amcl] [DEBUG] []: New pose:  0.013 -0.002  0.000 (laserReceived() at /home/ubuntu/tb2_demo_ws/src/ros2/navigation/amcl/src/amcl_node.cpp:1477)
[amcl] [INFO] []: Setting pose (1498865760.050656): 5.500 4.500 0.000 (handleInitialPoseMessage() at /home/ubuntu/tb2_demo_ws/src/ros2/navigation/amcl/src/amcl_node.cpp:1625)
[amcl] [DEBUG] []: Laser 0 angles in base frame: min: -0.510 inc: 0.003 (laserReceived() at /home/ubuntu/tb2_demo_ws/src/ros2/navigation/amcl/src/amcl_node.cpp:1329)
[amcl] [DEBUG] []: Num samples: 5000 (laserReceived() at /home/ubuntu/tb2_demo_ws/src/ros2/navigation/amcl/src/amcl_node.cpp:1371)
[amcl] [DEBUG] []: Max weight pose: 5.498 4.504 -0.008 (laserReceived() at /home/ubuntu/tb2_demo_ws/src/ros2/navigation/amcl/src/amcl_node.cpp:1425)
[amcl] [DEBUG] []: New pose:  5.498  4.504 -0.008 (laserReceived() at /home/ubuntu/tb2_demo_ws/src/ros2/navigation/amcl/src/amcl_node.cpp:1477)
[amcl] [DEBUG] []: Saving pose to server. x: 5.498, y: 4.504 (savePoseToServer() at /home/ubuntu/tb2_demo_ws/src/ros2/navigation/amcl/src/amcl_node.cpp:783)
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

Terminal 4:
```
. /opt/ros/kinetic/setup.bash
. ~/ros2_ws/install/setup.bash
rosrun map_server map_server `ros2 pkg prefix turtlebot2_amcl`/share/turtlebot2_amcl/examples/osrf_map.yaml
```

Add TF, PoseArray, LaserArray and Map displays, and you should see the particle cloud move around the map with the robot.

![example localization](https://github.com/ros2/turtlebot2_demo/raw/master/turtlebot2_amcl/doc/turtlebot2_amcl_rviz.png "Example of RViz visualization of localization")


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
