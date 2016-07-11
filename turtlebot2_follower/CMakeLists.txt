cmake_minimum_required(VERSION 3.5)

project(turtlebot2_follower)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

include_directories(include)

macro(targets)
  if(NOT target_suffix STREQUAL "")
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
  endif()

  add_executable(follower${target_suffix} src/follower.cpp)
  ament_target_dependencies(follower${target_suffix}
    "rclcpp${target_suffix}"
    "nav_msgs"
    "sensor_msgs")

  install(TARGETS follower${target_suffix}
          DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)
