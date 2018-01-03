##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_build)

##############################################################################
# Ament
##############################################################################

find_package(ament_cmake REQUIRED)

set(${PROJECT_NAME}_MODULES "")
list(APPEND ${PROJECT_NAME}_MODULES cmake/ecl_platform_detection.cmake)
list(APPEND ${PROJECT_NAME}_MODULES cmake/ecl_package.cmake)
list(APPEND ${PROJECT_NAME}_MODULES cmake/ecl_find_sse.cmake)
list(APPEND ${PROJECT_NAME}_MODULES cmake/ecl_build_utilities.cmake)
list(APPEND ${PROJECT_NAME}_MODULES cmake/ecl_cx11.cmake)
list(APPEND ${PROJECT_NAME}_MODULES cmake/cotire.cmake)

ament_package(
    CONFIG_EXTRAS ${${PROJECT_NAME}_MODULES}
)


##############################################################################
# Work
##############################################################################

install(DIRECTORY cmake
        DESTINATION share
)
