This repository contains the code and supporting files to run TurtleBot 2 demos using ROS 2. Due to reliance on existing Linux-only code and dependencies, these demos are intended for use only on Linux (that could change in the future).

We're assuming here that you have an Orbbec Astra depth camera. Extra work would be required to use the Kinect or Asus Xtion Pro. Without an Astra, you can still do joystick teleop.

# Installation

## Install some dependencies
```
sudo apt-get install ros-kinetic-kobuki-driver ros-kinetic-kobuki-ftdi ros-kinetic-common-msgs ros-kinetic-astra-camera libusb-1.0.0-dev libudev-dev
```
Reason for each dependency:
* `ros-kinetic-kobuki-driver` : our ROS 2 kobuki driver builds on top of this package (and its dependencies)
* `ros-kinetic-kobuki-ftdi` : we use a helper script from this package to install a `udev` rule
* `ros-kinetic-common-msgs` : to support use of the `ros1_bridge`, we need the ROS 1 messages available
* `ros-kinetic-astra-camera` : we're compiling our own ROS 2 fork of this package, so we don't actually need the ROS 1 version; we're installing it as a convenient way to ensure that all of its dependencies are installed

## Get the ROS 2 code
Follow the [usual instructions](https://github.com/ros2/ros2/wiki/Linux-Development-Setup#get-ros-20-code) for getting ROS 2 code, then add a couple of repositories:
```
cd ~/ros2_ws/src
git clone https://github.com/ros2/turtlebot2_demo.git
git clone https://github.com/ros2/ros_astra_camera.git
```

## Build the ROS 2 code, including the new nodes
```
# We'll eventually want to use the ROS 1 bridge, too
. /opt/ros/kinetic/setup.bash
cd ~/ros2_ws
./src/ament/ament_tools/scripts/ament.py build
```
## Configure a couple of things
```
# Astra device rule
cd ~/ros2_ws/src/ros_astra_camera
sudo cp 56-orbbec-usb.rules /etc/udev/rules.d
# copy across the udev rules for kobuki
rosrun kobuki_ftdi create_udev_rules
sudo service udev reload
sudo service udev restart
```

# Run the new nodes

## Joystick teleop
Try the launch file:
```
. ~/ros2_ws/install/setup.bash
launch ~/ros2_ws/src/turtlebot2_demo/turtlebot2_drivers/launch/turtlebot_joy.py
```

Or, run the nodes separately:
```
. ~/ros2_ws/install/setup.bash
kobuki_node
```
```
. ~/ros2_ws/install/setup.bash
joy_node
```

## Follower
Try the launch file:
```
. ~/ros2_ws/install/setup.bash
launch ~/ros2_ws/src/turtlebot2_demo/turtlebot2_drivers/launch/turtlebot_follow.py
```

Or, run the nodes separately. Note that here we're specifically requesting the OpenSplice variants of the nodes; this is to ensure that we use an RMW implementation that supports large message transfer, which is required for the depth image produced by the Astra camera. This constraint will be relaxed in the future.
```
. ~/ros2_ws/install/setup.bash
kobuki_node__rmw_opensplice_cpp
```
```
. ~/ros2_ws/install/setup.bash
astra_camera_node__rmw_opensplice_cpp
```
```
. ~/ros2_ws/install/setup.bash
follower__rmw_opensplice_cpp
```
