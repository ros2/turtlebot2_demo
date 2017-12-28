##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(kobuki_driver)

##############################################################################
# Catkin
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(ecl_build REQUIRED)
find_package(ecl_mobile_robot REQUIRED)
find_package(ecl_converters REQUIRED)
find_package(ecl_devices REQUIRED)
find_package(ecl_geometry REQUIRED)
find_package(ecl_sigslots REQUIRED)
find_package(ecl_time REQUIRED)
find_package(ecl_command_line REQUIRED)


##############################################################################
# Project Configuration
##############################################################################

ecl_enable_cxx11_compiler()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -fPIC  -Wno-deprecated-declarations")

include_directories(include)

##############################################################################
# Sources
##############################################################################

add_subdirectory(src)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
)


ament_export_libraries(kobuki)
ament_export_include_directories(include)
ament_export_dependencies(
	ecl_command_line
	ecl_converters
	ecl_devices
	ecl_geometry
	ecl_mobile_robot
	ecl_sigslots
	ecl_time
)
ament_package()
