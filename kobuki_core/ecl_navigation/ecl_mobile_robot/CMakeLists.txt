##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_mobile_robot)

##############################################################################
# Catkin
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(ecl_build REQUIRED)
find_package(ecl_errors REQUIRED)
find_package(ecl_geometry REQUIRED)
find_package(ecl_math REQUIRED)
find_package(ecl_formatters REQUIRED)
find_package(ecl_linear_algebra REQUIRED)


##############################################################################
# Project Configuration
##############################################################################

ecl_enable_cxx11_compiler()
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

include_directories(include)

##############################################################################
# Sources
##############################################################################

add_subdirectory(include)
add_subdirectory(src)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(
	ecl_build
	ecl_errors
	ecl_geometry
	ecl_math
	ecl_formatters
	ecl_linear_algebra
)

ament_package()
