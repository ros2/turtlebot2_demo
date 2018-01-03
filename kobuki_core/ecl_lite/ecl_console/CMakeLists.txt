##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_console)

##############################################################################
# Find
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(ecl_build REQUIRED)
find_package(ecl_config REQUIRED)

##############################################################################
# Exports
##############################################################################

include_directories(include include/ecl/console)
ament_export_include_directories(include)


ament_export_dependencies(
  ecl_build
  ecl_config
)

##############################################################################
# Project configuration
##############################################################################

ecl_enable_cxx11_compiler()

##############################################################################
# Installs
##############################################################################

install(
  DIRECTORY include/
  DESTINATION include/
	FILES_MATCHING PATTERN "*.hpp"
)

ament_package()
