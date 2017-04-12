cmake_minimum_required(VERSION 3.5)
project(depth_to_pointcloud)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)

find_package(image_geometry REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(depth_to_pointcloud_node
  src/depth_to_pointcloud_node.cpp
)

ament_target_dependencies(depth_to_pointcloud_node
  "image_geometry"
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS depth_to_pointcloud_node
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
)

ament_package()
