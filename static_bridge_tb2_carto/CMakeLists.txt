cmake_minimum_required(VERSION 3.5)

project(static_bridge_tb2_carto)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

find_package(rmw REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(ros1_bridge REQUIRED)

include(cmake/find_ros1_package.cmake)

find_package(PkgConfig REQUIRED)

find_ros1_package(roscpp REQUIRED)

add_executable(${PROJECT_NAME}
  src/static_bridge_tb2_carto.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "ros1_bridge"
  "ros1_roscpp"
  "ros1_std_msgs"
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION bin
)

ament_package()
