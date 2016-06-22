cmake_minimum_required(VERSION 3.5)

project(turtlebot2_drivers)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(kobuki_driver REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(${kobuki_driver_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS})
link_directories(${kobuki_driver_LIBRARY_DIRS} ${rclcpp_LIBRARY_DIRS} ${nav_msgs_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS})

add_executable(kobuki_node src/kobuki_node.cpp)
target_link_libraries(kobuki_node ${kobuki_driver_LIBRARIES} ${rclcpp_LIBRARIES} ${nav_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES})
add_executable(dumb_teleop src/dumb_teleop.cpp)
target_link_libraries(dumb_teleop ${kobuki_driver_LIBRARIES} ${rclcpp_LIBRARIES} ${nav_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES})

install(TARGETS kobuki_node dumb_teleop
        DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
