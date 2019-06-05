cmake_minimum_required(VERSION 3.5)

project(turtlebot2_drivers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
# FIXME: this should not be required, but somehow the lower layers are not
# exporting this properly so that we can depend on it
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_driver REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(kobuki_node src/kobuki_node.cpp)
ament_target_dependencies(kobuki_node
  "geometry_msgs"
  "kobuki_driver"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_ros")

target_include_directories(kobuki_node
  PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
)

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
