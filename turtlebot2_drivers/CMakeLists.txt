cmake_minimum_required(VERSION 3.5)

project(turtlebot2_drivers)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(kobuki_driver REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(kobuki_node src/kobuki_node.cpp)
ament_target_dependencies(kobuki_node
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_ros")
# Note that this setting must come after the call to
# ament_target_dependencies() to avoid including a ROS1 tf2 installation that
# might be on the system.
target_include_directories(kobuki_node PRIVATE ${kobuki_driver_INCLUDE_DIRS})
target_link_libraries(kobuki_node ${kobuki_driver_LIBRARIES})

add_executable(dumb_teleop src/dumb_teleop.cpp)
ament_target_dependencies(dumb_teleop
  "rclcpp"
  "geometry_msgs")

install(TARGETS
  kobuki_node
  dumb_teleop
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  PROGRAMS
  launch/turtlebot_carto_2d.py
  launch/turtlebot_carto_3d.py)

ament_package()
