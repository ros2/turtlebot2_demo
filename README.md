This repository will contain the code and supporting files to run TurtleBot 2 demos using ROS 2.

# Installation

## Install some dependencies
```
sudo apt-get install ros-kinetic-kobuki-driver 
. /opt/ros/kinetic/setup.bash
```

## Get the ROS 2 code
Follow the usual instructions for getting ROS 2 code, then add this repository:
```
cd ~/ros2_ws/src
git clone git@github.com:ros2/turtlebot2_demo.git
```

Also, get on certain branches:
```
cd ~/ros2_ws/src/ros2/common_interfaces
git checkout enable_msgs
```

## Build the ROS 2 code, including the new nodes
```
. /opt/ros/kinetic/setup.bash
cd ~/ros2_ws
./src/ament/ament_tools/scripts/ament.py build
```

## Run the new nodes

The kobuki node can be run like so:

```
. ~/ros2_ws/install/setup.bash
kobuki_node
```

