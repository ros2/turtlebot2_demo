##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_filesystem)

##############################################################################
# Thread Detection
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(ecl_build REQUIRED)
find_package(ecl_config REQUIRED)
find_package(ecl_errors REQUIRED)


# This finds the platform specific filesystem libraries and sets them in the
# variable ECL_PLATFORM_FILESYSTEM_LIBRARIES. Right now this is super simple
# (typical just 'rt' on linux), but if this gets more complicated then
# appropriate dependencies need to be listed in catkin_package's DEPENDS.
ecl_detect_filesystem()

##############################################################################
# Ament
##############################################################################

include_directories(include)

##############################################################################
# Actual project configuration
##############################################################################

add_subdirectory(include)
add_subdirectory(src)

ament_export_libraries(${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(ecl_config ecl_errors)

ament_package()
