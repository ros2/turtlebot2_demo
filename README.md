This repository will contain the code and supporting files to run TurtleBot 2 demos using ROS 2.

# Installation

## Install some dependencies
```
sudo apt-get install ros-kinetic-kobuki-driver ros-kinetic-kobuki-ftdi ros-kinetic-common-msgs ros-kinetic-astra-camera
```

## Get the ROS 2 code
Follow the usual instructions for getting ROS 2 code, then add a couple of repositories:
```
cd ~/ros2_ws/src
git clone git@github.com:ros2/turtlebot2_demo.git
git clone git@github.com:ros2/ros_astra_camera.git
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

## Kobuki
```
. ~/ros2_ws/install/setup.bash
kobuki_node
```

## Joystick control
```
. ~/ros2_ws/install/setup.bash
joy_node
```

## Astra camera
```
. ~/ros2_ws/install/setup.bash
astra_camera_node
```

## Follower
```
. ~/ros2_ws/install/setup.bash
follower
```
