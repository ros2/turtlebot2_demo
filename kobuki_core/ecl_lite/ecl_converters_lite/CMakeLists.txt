##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_converters_lite)

##############################################################################
# Ament
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(ecl_config REQUIRED)

include_directories(include include/ecl/converters_lite)
ament_export_include_directories(include)

##############################################################################
# Actual project configuration
##############################################################################

add_subdirectory(include)
add_subdirectory(src)

ament_package()
