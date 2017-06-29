cmake_minimum_required(VERSION 3.5)

project(turtlebot2_drivers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
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
  "geometry_msgs"
  "rclcpp"
)

install(TARGETS
  kobuki_node
  dumb_teleop
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
