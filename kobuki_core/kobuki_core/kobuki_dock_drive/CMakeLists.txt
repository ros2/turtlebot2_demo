##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(kobuki_dock_drive)

##############################################################################
# Find Packages
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(ecl_build REQUIRED)
find_package(ecl_threads REQUIRED)
find_package(ecl_geometry REQUIRED)
find_package(ecl_linear_algebra REQUIRED)


##############################################################################
# Project Configuration
##############################################################################

ecl_enable_cxx11_compiler()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC  -Wno-deprecated-declarations")

include_directories(include )

##############################################################################
# Sources
##############################################################################

add_library(kobuki_dock_drive SHARED src/dock_drive.cpp src/dock_drive_states.cpp src/dock_drive_debug.cpp)
ament_target_dependencies(kobuki_dock_drive ecl_errors ecl_config ecl_geometry)

##############################################################################
# Installs
##############################################################################

install(TARGETS kobuki_dock_drive
        DESTINATION lib
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
)


ament_export_libraries(kobuki_dock_drive)
ament_export_include_directories(include)
ament_export_dependencies(ecl_threads ecl_geometry ecl_linear_algebra)

ament_package()
