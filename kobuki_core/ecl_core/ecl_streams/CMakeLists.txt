##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_streams)

##############################################################################
# Ament
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(ecl_errors REQUIRED)
find_package(ecl_concepts REQUIRED)
find_package(ecl_devices REQUIRED)
find_package(ecl_time REQUIRED)
find_package(ecl_converters REQUIRED)
find_package(ecl_type_traits REQUIRED)

include_directories(include )

##############################################################################
# Actual project configuration
##############################################################################

add_subdirectory(include)
add_subdirectory(src)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(
	ecl_errors
	ecl_concepts
	ecl_devices
	ecl_time
	ecl_converters
	ecl_type_traits
)

ament_package()
