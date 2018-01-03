##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_devices)

##############################################################################
# Ament
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(ecl_config REQUIRED)
find_package(ecl_errors REQUIRED)
find_package(ecl_mpl REQUIRED)
find_package(ecl_type_traits REQUIRED)
find_package(ecl_utilities REQUIRED)
find_package(ecl_containers REQUIRED)
find_package(ecl_threads REQUIRED)


include_directories(include )

##############################################################################
# Actual project configuration
##############################################################################

ecl_enable_cxx11_compiler()

add_subdirectory(include)
add_subdirectory(src)


ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(  ecl_config
	ecl_errors
	ecl_mpl
	ecl_type_traits
	ecl_utilities
	ecl_containers
	ecl_threads
)

ament_package()
