##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_threads)

##############################################################################
# Thread Detection
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(ecl_build REQUIRED)
find_package(ecl_config REQUIRED)
find_package(ecl_errors REQUIRED)
find_package(ecl_concepts REQUIRED)
find_package(ecl_exceptions REQUIRED)
find_package(ecl_time REQUIRED)
find_package(ecl_utilities REQUIRED)

ecl_detect_threads()

set (thread_INCLUDE_DIRS "" CACHE STRING "Platform thread include directories")
if(ECL_PLATFORM_HAS_POSIX_THREADS)
  set (thread_LIBRARIES "pthread" CACHE STRING "Platform thread libraries")
else()
  set (thread_LIBRARIES "" CACHE STRING "Platform thread libraries")
endif()

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
ament_export_dependencies(  ecl_config
	ecl_errors
	ecl_concepts
	ecl_exceptions
	ecl_time
	ecl_utilities
)

ament_package()
