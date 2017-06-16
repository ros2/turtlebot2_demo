cmake_minimum_required(VERSION 3.5)

project(turtlebot2_follower)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(follower src/follower.cpp)
ament_target_dependencies(follower
  "rclcpp"
  "nav_msgs"
  "sensor_msgs")

install(TARGETS follower
        DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
