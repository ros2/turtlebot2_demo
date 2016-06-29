cmake_minimum_required(VERSION 3.5)

project(turtlebot2_follower)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include ${rclcpp_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS})
link_directories(${rclcpp_LIBRARY_DIRS} ${nav_msgs_LIBRARY_DIRS} ${sensor_msgs_LIBRARY_DIRS})

add_executable(follower src/follower.cpp)
target_link_libraries(follower ${rclcpp_LIBRARIES} ${nav_msgs_LIBRARIES} ${sensor_msgs_LIBRARIES})

install(TARGETS follower
        DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
