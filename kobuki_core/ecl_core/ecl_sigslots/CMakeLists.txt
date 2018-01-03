##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_sigslots)

##############################################################################
# Ament
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(ecl_config REQUIRED)
find_package(ecl_threads REQUIRED)

include_directories(include)

##############################################################################
# Actual project configuration
##############################################################################

add_subdirectory(include)
add_subdirectory(src)

ament_export_include_directories(include)
ament_export_dependencies(ecl_config ecl_threads)

ament_package()
