cmake_minimum_required(VERSION 3.5)
project(depth_to_pointcloud)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(image_geometry REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(depth_to_pointcloud_node
  src/depth_to_pointcloud_node.cpp
)

ament_target_dependencies(depth_to_pointcloud_node
  "image_geometry"
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS depth_to_pointcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
