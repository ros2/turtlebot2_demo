##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_ipc)

##############################################################################
# Catkin
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(ecl_config REQUIRED)
find_package(ecl_errors REQUIRED)
find_package(ecl_build REQUIRED)
find_package(ecl_time_lite REQUIRED)
find_package(ecl_exceptions REQUIRED)
find_package(ecl_time REQUIRED)



include_directories(include)

##############################################################################
# Actual project configuration
##############################################################################

add_subdirectory(include)
add_subdirectory(src)

ecl_detect_threads()

if(ECL_PLATFORM_HAS_POSIX_THREADS)
		ament_export_include_directories(include)
		ament_export_libraries(${PROJECT_NAME})
		ament_export_dependencies(
				ecl_config
				ecl_errors
				ecl_time_lite
				ecl_exceptions
				ecl_time
		)
else()
		ament_export_include_directories(include)
		ament_export_dependencies(
				ecl_config
				ecl_errors
				ecl_time_lite
				ecl_exceptions
				ecl_time
		)
endif()

ament_package()
