cmake_minimum_required(VERSION 3.5)

project(turtlebot2_drivers)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(kobuki_driver REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${kobuki_driver_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
add_executable(kobuki_node src/kobuki_node.cpp)
link_directories(${kobuki_driver_LIBRARY_DIRS} ${rclcpp_LIBRARY_DIRS} ${std_msgs_LIBRARY_DIRS})
target_link_libraries(kobuki_node ${kobuki_driver_LIBRARIES} ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})
install(TARGETS kobuki_node
        DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
