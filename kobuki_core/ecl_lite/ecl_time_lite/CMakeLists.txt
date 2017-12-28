##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_time_lite)

##############################################################################
# Ament
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(ecl_build REQUIRED)
find_package(ecl_config REQUIRED)
find_package(ecl_errors REQUIRED)


##############################################################################
# Platform Configuration
##############################################################################

# This configures ECL_PLATFORM_TIME_LIBRARIES and
# other ECL_PLATFORM_HAS_XXX flags
ecl_detect_timers()

##############################################################################
# Exports
##############################################################################


include_directories(include)

##############################################################################
# CMake File Configuration
##############################################################################

# Make sure we pick up cmake configured files
#include_directories(include)

##############################################################################
# Actual project configuration
##############################################################################

add_subdirectory(include)
add_subdirectory(src)

ament_export_include_directories(include)
#ament_export_dependencies(ECL_PLATFORM_TIME)
ament_export_libraries(${PROJECT_NAME})

ament_package()
