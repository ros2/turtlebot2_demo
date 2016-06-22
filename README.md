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

## Install some dependencies
```
sudo apt-get install ros-kinetic-catkin ros-kinetic-cmake-modules libftdi-dev ros-kinetic-sophus
. /opt/ros/kinetic/setup.bash
```

## Make a workspace and get some code
```
mkdir ws
cd ws
mkdir src
cd src
git clone https://github.com/stonier/ecl_core.git
git clone https://github.com/stonier/ecl_lite.git
git clone https://github.com/stonier/ecl_navigation.git
git clone https://github.com/stonier/ecl_tools.git
git clone https://github.com/yujinrobot/kobuki_core.git
```

## Build and install the workspace
```
cd ws
catkin_make --only-pkg-with-deps kobuki_driver
```

