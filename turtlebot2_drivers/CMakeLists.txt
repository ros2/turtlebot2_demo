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
find_package(rmw_implementation_cmake REQUIRED)

include_directories(${kobuki_driver_INCLUDE_DIRS})

macro(targets)
  if(NOT target_suffix STREQUAL "")
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
  endif()
  add_executable(kobuki_node${target_suffix} src/kobuki_node.cpp)
  ament_target_dependencies(kobuki_node${target_suffix}
    "rclcpp${target_suffix}"
    "nav_msgs"
    "geometry_msgs")
  target_link_libraries(kobuki_node${target_suffix} ${kobuki_driver_LIBRARIES})
  add_executable(dumb_teleop${target_suffix} src/dumb_teleop.cpp)
  ament_target_dependencies(dumb_teleop${target_suffix}
    "rclcpp${target_suffix}"
    "nav_msgs"
    "geometry_msgs")

  install(TARGETS kobuki_node${target_suffix} dumb_teleop${target_suffix}
          DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)
