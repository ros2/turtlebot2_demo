##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_math)

##############################################################################
# Ament
##############################################################################


find_package(ament_cmake REQUIRED)
find_package(ecl_type_traits REQUIRED)

include_directories(include)

##############################################################################
# Actual project configuration
##############################################################################

add_subdirectory(include)
add_subdirectory(src)

ament_export_include_directories(include)
ament_export_dependencies(ecl_type_traits)

ament_package()
