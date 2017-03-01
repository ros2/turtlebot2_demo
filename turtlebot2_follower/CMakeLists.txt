cmake_minimum_required(VERSION 3.5)

project(turtlebot2_follower)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
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
