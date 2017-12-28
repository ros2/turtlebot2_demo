##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_geometry)

##############################################################################
# Ament
##############################################################################


find_package(ament_cmake REQUIRED)
find_package(ecl_build REQUIRED)
find_package(ecl_config REQUIRED)
find_package(ecl_type_traits REQUIRED)
find_package(ecl_containers REQUIRED)
find_package(ecl_formatters REQUIRED)
find_package(ecl_linear_algebra REQUIRED)
find_package(ecl_exceptions REQUIRED)
find_package(ecl_math REQUIRED)
find_package(ecl_mpl REQUIRED)


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
	ecl_config
	ecl_type_traits
	ecl_containers
	ecl_formatters
	ecl_linear_algebra
	ecl_exceptions
	ecl_math
	ecl_mpl
)

ament_package()
