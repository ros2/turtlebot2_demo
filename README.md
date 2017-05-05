This repository contains the code and supporting files to run TurtleBot 2 demos using ROS 2. Due to reliance on existing Linux-only code and dependencies, these demos are intended for use only on Linux (that could change in the future).

# Disclaimer

This demo is under development and these instructions may not work out of the box and will likely change in the near future

<!-- Here's a video of the very first successful run of ROS 2 follower: https://www.youtube.com/watch?v=YTlls9yHZog.

This is very much a work in progress and there's plenty of functionality present in the ROS 1 equivalent system that's currently missing or disabled, but it's an important step. -->

This demo assumes that you have an Orbbec Astra depth camera. Extra work would be required to use the Kinect or Asus Xtion Pro. Without an Astra, you can still do joystick teleop.

# Installation

## Prerequisites:

- ROS kinetic installed
- A ROS2 workspace installed [from source](https://github.com/ros2/ros2/wiki/Linux-Development-Setup) or [from binary](https://github.com/ros2/ros2/wiki/Linux-Install-Binary) and sourced in your terminal.
## Install some dependencies
```
sudo apt-get install libboost-iostreams-dev libboost-regex-dev libboost-system-dev libboost-thread-dev libsdl1.2-dev libsdl-image1.2-dev libudev-dev libusb-1.0.0-dev libyaml-cpp-dev ros-kinetic-astra-camera ros-kinetic-common-msgs ros-kinetic-kobuki-driver ros-kinetic-kobuki-ftdi
```
Reason for each dependency:
* `ros-kinetic-kobuki-driver` : our ROS 2 kobuki driver builds on top of this package (and its dependencies)
* `ros-kinetic-kobuki-ftdi` : we use a `udev` rule from this package
* `ros-kinetic-common-msgs` : to support use of the `ros1_bridge`, we need the ROS 1 messages available (TODO: document use of the bridge to view depth images and other stuff)
* `ros-kinetic-astra-camera` : we're compiling our own ROS 2 fork of this package, so we don't actually need the ROS 1 version; we're installing it as a convenient way to ensure that all of its dependencies are installed

## Get the demo code
```
mkdir ~/turtlebot_demo_ws/src
cd ~/turtlebot_demo_ws
wget https://raw.githubusercontent.com/ros2/turtlebot2_demo/master/turtlebot2_demo.repos
vcs import src < turtlebot2_demo.repos
```

## Build the ROS 2 code, including the new nodes
```
# We'll eventually want to use the ROS 1 bridge, too
. /opt/ros/kinetic/setup.bash
cd ~/turtlebot_demo_ws
./src/ament/ament_tools/scripts/ament.py build -s
```
## Configure a couple of things
```
# Astra device rule
cd ~/ros2_ws/src/ros_astra_camera
sudo cp 56-orbbec-usb.rules /etc/udev/rules.d
# Kobuki device rule
sudo cp `rospack find kobuki_ftdi`/57-kobuki.rules /etc/udev/rules.d
sudo service udev reload
sudo service udev restart
```

# Run the new nodes

## Joystick teleop
Try the launch file:
```
. ~/turtlebot_demo_ws/install/setup.bash
launch ~/turtlebot_demo_ws/src/turtlebot2_demo/turtlebot2_drivers/launch/turtlebot_joy.py
```

Or, run the nodes separately:
```
. ~/turtlebot_demo_ws/install/setup.bash
kobuki_node
```
```
. ~/turtlebot_demo_ws/install/setup.bash
joy_node
```

## Follower
Try the launch file:
```
. ~/turtlebot_demo_ws/install/setup.bash
launch ~/turtlebot_demo_ws/src/turtlebot2_demo/turtlebot2_drivers/launch/turtlebot_follow.py
```

Or, run the nodes separately.
```
. ~/turtlebot_demo_ws/install/setup.bash
kobuki_node
```
```
. ~/turtlebot_demo_ws/install/setup.bash
astra_camera_node
```
```
. ~/turtlebot_demo_ws/install/setup.bash
follower
```

# Discussion
What's happening here compared to the ROS 1 versions of these demos? Well, it's 100% ROS 2, with no bridge or shim. We took 4 different
approaches in building the different pieces:

1. Kobuki driver: we wrote a new, very small rclcpp node that calls into the existing kobuki driver packages, which are organized to be roscpp-independent. In this case, we're building on top of ROS 1 packages, but they don't use `roscpp` or other parts of the ROS 1 middleware, so we're just using them as supporting libraries.
2. Astra driver: we forked and ported the existing ROS 1 package (there's no roscpp-independent package separation).
3. Joystick driver: we wrote a simple rclcpp node from scratch (Linux-only for now).
4. Follower node: we created a new package into which we copied and then ported the ROS 1 follower nodelet.

As we start migrating more code to ROS 2, we'll discover more about these kinds of techniques and arrive at some best practices that we can recommend for similar projects.
