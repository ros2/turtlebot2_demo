cmake_minimum_required(VERSION 3.5)

project(joy)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${rclcpp_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS})
link_directories(${rclcpp_LIBRARY_DIRS} ${sensor_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS})

add_executable(joy_node joy_node.cpp)
target_link_libraries(joy_node ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES})

install(TARGETS joy_node DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
