This repository contains the code and supporting files to run TurtleBot 2 demos using ROS 2. Due to reliance on existing Linux-only code and dependencies, these demos are intended for use only on Linux (that could change in the future).

<!-- Here's a video of the very first successful run of ROS 2 follower: https://www.youtube.com/watch?v=YTlls9yHZog.

This is very much a work in progress and there's plenty of functionality present in the ROS 1 equivalent system that's currently missing or disabled, but it's an important step. -->

This demo assumes that you have an Orbbec Astra depth camera. Extra work would be required to use the Kinect or Asus Xtion Pro. Without an Astra, you can still do joystick teleop.
For instructions on how to setup your turtlebot please see [Setup your turtlebot2](https://github.com/ros2/turtlebot2_demo/blob/master/Turtlebot_Setup.md)

# Installation

## Installation from binaries

First, install ROS2 from binaries following [these instructions](https://github.com/ros2/ros2/wiki/Linux-Install-Debians)

Then install the turtlebo2 demo specific packages:
```
sudo apt install ros-r2b2-turtlebot2* ros-kinetic-kobuki-ftdi
```

## Installation from source
This assumes that you have ROS Kinetic installed (or at lease have the ros apt repository in your sources)

First, install ROS2 from source following [these instructions](https://github.com/ros2/ros2/wiki/Linux-Development-Setup)

Then get the turtlebo2 demos specific code:
```
cd <YOUR_ROS2_WORKSPACE>
wget https://raw.githubusercontent.com/ros2/turtlebot2_demo/release-latest/turtlebot2_demo.repos
vcs import src < turtlebot2_demo.repos
```

### Install some dependencies:
```bash
sudo apt-get install --no-install-recommends -y libboost-iostreams-dev libboost-regex-dev libboost-system-dev libboost-thread-dev libceres-dev libgoogle-glog-dev liblua5.2-dev libpcl-dev libprotobuf-dev libsdl1.2-dev libsdl-image1.2-dev libsuitesparse-dev libudev-dev libusb-1.0.0-dev libyaml-cpp-dev protobuf-compiler python-sphinx ros-kinetic-kobuki-driver ros-kinetic-kobuki-ftdi
```

Reason for each dependency:
* `ros-kinetic-kobuki-driver` : our ROS 2 kobuki driver builds on top of this package (and its dependencies)
* `ros-kinetic-kobuki-ftdi` : we use a `udev` rule from this package
* `ros-kinetic-common-msgs` : to support use of the `ros1_bridge`, we need the ROS 1 messages available (TODO: document use of the bridge to view depth images and other stuff)
* `ros-kinetic-astra-camera` : we're compiling our own ROS 2 fork of this package, so we don't actually need the ROS 1 version; we're installing it as a convenient way to ensure that all of its dependencies are installed

### Build the ros2 code

For resource constrained platforms we will split the build into 2 steps to make sure not to overflow the memory
```bash
src/ament/ament_tools/scripts/ament.py build --isolated --symlink-install --parallel --skip-packages cartographer cartographer_ros ros1_bridge turtlebot2_amcl turtlebot2_drivers turtlebot2_follower turtlebot2_cartographer turtlebot2_teleop
```

Now the resource intensive packages and the ones depending on ROS1 packages:
```bash
source /opt/ros/kinetic/setup.bash
src/ament/ament_tools/scripts/ament.py build --isolated --symlink-install --parallel --only cartographer cartographer_ros turtlebot2_amcl turtlebot2_cartographer turtlebot2_drivers turtlebot2_follower turtlebot2_teleop --make-flags -j2 -l2
```
Go grab a coffee (or a meal if you compile on ARM)

# Configure a couple of things

## Setup the udev rules

### Copy the astra udev rules

#### If you installed from binaries
If you installed from binary you'll need to download the udev rule by hand:
```bash
wget https://raw.githubusercontent.com/ros2/ros_astra_camera/ros2/56-orbbec-usb.rules
```

#### If you installed from source
```bash
cd ~/ros2_ws/src/ros_astra_camera
```

#### Copy the rules file
```bash
sudo cp 56-orbbec-usb.rules /etc/udev/rules.d
```

### Copy the kobuki udev rule

```bash
sudo cp `rospack find kobuki_ftdi`/57-kobuki.rules /etc/udev/rules.d
```

### Restart the udev service
```bash
sudo service udev reload
sudo service udev restart
```

## Source your workspace

If installed from Debian packages
```bash
source /opt/ros/kinetic/setup.bash
source /opt/ros/r2b2/setup.bash
```

If installed from source
```bash
source /opt/ros/kinetic/setup.bash
source ~/ros2_ws/install/local_setup.bash
```

You'll need to do this step for every terminal you use for these demos

# Run the demos

## Joystick teleop
This is a classic teleoperation demo where the robot can be driven around using a gamepad controller. Thie demo has been tested with logitech controllers and uses `RB` as a deadman, the left joystick for driving forward/backward and the right joystick for rotation.
Try the launch file:
```
launch `ros2 pkg prefix turtlebot2_teleop`/share/turtlebot2_teleop/launch/turtlebot_joy.py
```

Or, run the nodes separately:
```
ros2 run turtlebot2_drivers kobuki_node
```
```
ros2 run joy joy_node
```

Note: this demo assumes that your controller is in D mode (switch on the back) and that the MODE led is on. 

## Follower
This demo uses the astra camera to detect blobs in the depthimage and follow them
Try the launch file:
```
launch `ros2 pkg prefix turtlebot2_follower`/share/turtlebot2_follower/launch/turtlebot_follow.py
```

Or, run the nodes separately.
```
ros2 run turtlebot2_drivers kobuki_node
```
```
ros2 run astra_camera astra_camera_node -- -dw 320 -dh 240 -C -I
```
```
ros2 run turtlebot2_follower follower
```

## Cartographer (mapping)
This demo is using Google cartographer to build a map of the environment. The resulting map can be visualized in RViz using the ros1_bridge (more information below).

### Run the demo
Try the launch file:
```
launch `ros2 pkg prefix turtlebot2_cartographer`/share/turtlebot2_cartographer/launch/turtlebot_carto_2d.py
```

### Visualize the results
The created map can be visualized in Rviz on a remote computer by using the dynamic bridge that converts messages between ROS1 and ROS2.
This assumes that you have a ROS2 dynamic bridge on your system.

#### Installing the bridge

##### From binaries:
Setup your sources as explained on the [setup sources section](https://github.com/ros2/ros2/wiki/Linux-Install-Debians#setup-sources) and then run
```bash
sudo apt update && sudo apt install ros-r2b2-ros1-bridge
```

##### From source:
Build your ROS2 workspace as explained in [these instructions](https://github.com/ros2/ros1_bridge/blob/master/README.md#build-the-bridge-from-source).

#### Run the bridge

Terminal A:
```bash
. /opt/ros/kinetic/setup.bash
roscore
```

Terminal B:
```bash
. /opt/ros/kinetic/setup.bash
. <YOUR_ROS2_WORKSPACE>
ros2 run ros1_bridge dynamic_bridge
```

Terminal C:
```bash
. /opt/ros/kinetic/setup.bash
rosrun rviz rviz
```
Topics you can visualize in Rviz:
- the map on the topic `/map`
- the transforms on the topic `/tf`
- the depth images on the topic `/depth`
- the laserscans on the topic `/scans`

## AMCL (localization)
See the [AMCL demo README](https://github.com/ros2/turtlebot2_demo/blob/master/turtlebot2_amcl/README.md)

# Discussion
What's happening here compared to the ROS 1 versions of these demos? Well, it's 100% ROS 2, with no bridge or shim. We took 4 different
approaches in building the different pieces:

1. Kobuki driver: we wrote a new, very small rclcpp node that calls into the existing kobuki driver packages, which are organized to be roscpp-independent. In this case, we're building on top of ROS 1 packages, but they don't use `roscpp` or other parts of the ROS 1 middleware, so we're just using them as supporting libraries.
2. Astra driver: we forked and ported the existing ROS 1 package (there's no roscpp-independent package separation).
3. Joystick driver: we wrote a simple rclcpp node from scratch (Linux-only for now).
4. Follower node: we created a new package into which we copied and then ported the ROS 1 follower nodelet.

As we start migrating more code to ROS 2, we'll discover more about these kinds of techniques and arrive at some best practices that we can recommend for similar projects.
