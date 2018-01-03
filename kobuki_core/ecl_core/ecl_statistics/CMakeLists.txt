##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_statistics)

##############################################################################
# Ament
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(ecl_config REQUIRED)
find_package(ecl_build REQUIRED)
find_package(ecl_linear_algebra REQUIRED)
find_package(ecl_mpl REQUIRED)
find_package(ecl_type_traits REQUIRED)

##############################################################################
# Project Configuration
##############################################################################

ecl_enable_cxx11_compiler()
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

include_directories(include )

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
        ecl_linear_algebra
        ecl_mpl
        ecl_type_traits
)

ament_package()
