cmake_minimum_required(VERSION 3.5)

project(tb2_demo_teleop)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch/
  DESTINATION lib/${PROJECT_NAME})

ament_package()
