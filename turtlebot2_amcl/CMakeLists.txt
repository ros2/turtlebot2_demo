cmake_minimum_required(VERSION 3.5)

project(turtlebot2_amcl)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
  examples
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
