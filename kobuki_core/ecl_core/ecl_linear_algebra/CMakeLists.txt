##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_linear_algebra)

##############################################################################
# Ament
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(ecl_build REQUIRED)
find_package(ecl_eigen REQUIRED)
find_package(ecl_exceptions REQUIRED)
find_package(ecl_formatters REQUIRED)
find_package(ecl_math REQUIRED)
find_package(sophus REQUIRED)


find_package(Eigen 3 QUIET)
if(NOT Eigen_FOUND)
  include(${PROJ_SOURCE_DIR}cfg/FindEigen3.cmake)
  set(Eigen_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})
endif()
include_directories(${Eigen_INCLUDE_DIR})

##############################################################################
# Project Configuration
##############################################################################

ecl_enable_cxx11_compiler()
include_directories(include ${sophus_INCLUDE_DIRS} ${Eigen_INCLUDE_DIR})

##############################################################################
# Sources
##############################################################################

add_subdirectory(include)
add_subdirectory(src)

ament_export_libraries(${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(
	sophus
	ecl_build
	ecl_eigen
	ecl_exceptions
	ecl_formatters
	ecl_math
	)

ament_package()
