cmake_minimum_required(VERSION 3.5)
project(depthimage_to_pointcloud2)

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

add_executable(depthimage_to_pointcloud2_node
  src/depthimage_to_pointcloud2_node.cpp
)

ament_target_dependencies(depthimage_to_pointcloud2_node
  "image_geometry"
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS depthimage_to_pointcloud2_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
