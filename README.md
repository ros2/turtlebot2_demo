This repository will contain the code and supporting files to run TurtleBot 2 demos using ROS 2.

# Installation

## Install some dependencies
```
sudo apt-get install ros-kinetic-kobuki-driver ros-kinetic-common-msgs ros-kinetic-astra-camera
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

# Run the new nodes

## Kobuki
```
. ~/ros2_ws/install/setup.bash
kobuki_node
```

## Joystick control
```
. ~/ros2_ws/install/setup.bash
joy
```

## Astra camera
```
. ~/ros2_ws/install/setup.bash
astra_camera_noda
```

## Follower
```
. ~/ros2_ws/install/setup.bash
follower
```



