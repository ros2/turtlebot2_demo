##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_utilities)

##############################################################################
# Ament
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(ecl_mpl REQUIRED)
find_package(ecl_concepts REQUIRED)


include_directories(include)


##############################################################################
# Actual project configuration
##############################################################################

add_subdirectory(include)
add_subdirectory(src)

ament_export_include_directories(include)
ament_export_dependencies(ecl_mpl ecl_concepts)

ament_package()
