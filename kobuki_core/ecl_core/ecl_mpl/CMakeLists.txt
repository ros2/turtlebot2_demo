##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_mpl)

##############################################################################
# Ament
##############################################################################

find_package(ament_cmake REQUIRED)

include_directories(
  include
)

ament_export_include_directories(include)

##############################################################################
# Actual project configuration
##############################################################################

add_subdirectory(include)
add_subdirectory(src)

ament_package()
