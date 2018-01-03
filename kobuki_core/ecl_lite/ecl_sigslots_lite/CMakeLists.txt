##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_sigslots_lite)

##############################################################################
# Ament
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(ecl_config REQUIRED)
find_package(ecl_errors REQUIRED)

ament_export_include_directories(include)
include_directories(include include/ecl/sigslots_lite)

##############################################################################
# Actual project configuration
##############################################################################

add_subdirectory(include)
add_subdirectory(src)


ament_package()
