cmake_minimum_required(VERSION 3.5)

project(turtlebot2_localization)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
  examples/
  DESTINATION share/${PROJECT_NAME}/examples)

install(
  FILES
  launch/turtlebot2_amcl.py
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
