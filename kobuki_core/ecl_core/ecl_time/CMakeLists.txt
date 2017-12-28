##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_time)

##############################################################################
# Find Packages
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(ecl_config REQUIRED)
find_package(ecl_errors REQUIRED)
find_package(ecl_exceptions REQUIRED)
find_package(ecl_time_lite REQUIRED)
find_package(ecl_build REQUIRED)

##############################################################################
# Ament
##############################################################################

include_directories(include)

##############################################################################
# Project configuration
##############################################################################


ecl_enable_cxx11_compiler()

##############################################################################
# Sources
##############################################################################

add_subdirectory(include)
add_subdirectory(src)

ament_export_libraries(${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(ecl_config ecl_errors ecl_exceptions ecl_time_lite ecl_build)

ament_package()
