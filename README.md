This repository will contain the code and supporting files to run TurtleBot 2 demos using ROS 2.

# Installation
We need to get some catkin packages installed. Specifically, we need:

* `ecl_license`
* `ecl_build`
* `ecl_command_line`
* `ecl_config`
* `ecl_eigen`
* `ecl_errors`
* `ecl_exceptions`
* `ecl_mpl`
* `ecl_time_lite`
* `ecl_time`
* `ecl_type_traits`
* `ecl_concepts`
* `ecl_converters`
* `ecl_formatters`
* `ecl_math`
* `ecl_utilities`
* `ecl_containers`
* `ecl_threads`
* `ecl_devices`
* `ecl_sigslots`
* `ecl_linear_algebra`
* `ecl_geometry`
* `ecl_mobile_robot`
* `kobuki_driver`

Extend this list as we bring in other dependencies.

We're going to install them into their own tree, source the setup file from that install tree, then build our ROS 2 code on top.

## Install some dependencies
```
sudo apt-get install ros-kinetic-catkin ros-kinetic-cmake-modules libftdi-dev ros-kinetic-sophus
. /opt/ros/kinetic/setup.bash
```

Extend this list as we bring in other dependencies (or perhaps document using `rosdep` to do this step).

**Note**: The above line may require you to uninstall `python3-catkin-pkg`,
which may have a negative impact on other things that you're doing.

## Make a workspace and get some code
```
mkdir ~/catkin_ws
cd ~/catkin_ws
mkdir src
cd src
git clone https://github.com/stonier/ecl_core.git
git clone https://github.com/stonier/ecl_lite.git
git clone https://github.com/stonier/ecl_navigation.git
git clone https://github.com/stonier/ecl_tools.git
git clone https://github.com/yujinrobot/kobuki_core.git
```

Extend this list of repos as we bring in other dependencies (or perhaps create and reference a vcs/wstool/rosinstall/bash file).

## Build and install the workspace
```
. /opt/ros/kinetic/setup.bash
cd ~/catkin_ws
catkin_make install --only-pkg-with-deps kobuki_driver
```

## Get the ROS 2 code
Follow the usual instructions for getting ROS 2 code, then add this repository:
```
cd ~/ros2_ws/src
git clone git@github.com:ros2/turtlebot2_demo.git
```

Also:
```
cd ~/ros2_ws/src/ros2/common_interfaces
git checkout enable_msgs
```

## Build the ROS 2 code, including the new nodes
```
. ~/catkin_ws/install/setup.bash
cd ~/ros2_ws
./src/ament/ament_tools/scripts/ament.py build
```

## Run the new nodes

The kobuki node can be run like so:

```
. ~/ros2_ws/install/setup.bash
kobuki_node
```

