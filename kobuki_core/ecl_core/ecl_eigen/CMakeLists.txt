##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_eigen)

##############################################################################
# Ament
##############################################################################

find_package(ament_cmake REQUIRED)


##############################################################################
# Internal vs External Eigen
##############################################################################


#if(DEFINED ECL_EIGEN_IS_INTERNAL_3_2_1)
    message(STATUS "Using ecl's internal eigen 3.2.1.")
		ament_export_include_directories(include)

#elseif(DEFINED ECL_EIGEN_IS_INTERNAL_3_1_2)
#    set(ECL_EIGEN_IS_INTERNAL_3_1_2 1)
#    message(STATUS "Using ecl's internal eigen.")
#		ament_export_include_directories(include include/eigen3.1.2 include/eigen3.1.2/Eigen)
#
#else()
#    find_package(Eigen3 REQUIRED)
#    set(ECL_EIGEN_IS_EXTERNAL 1 CACHE BOOL "Using the system's eigen3 implementation.")
#    if(EIGEN3_FOUND)
#        # On trusty, it sets EIGEN3_INCLUDE_DIR but not EIGEN3_INCLUDE_DIRS (xenial is fine), so help it
#        if(NOT EIGEN3_INCLUDE_DIRS)
#          set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
#        endif()
#
#				ament_export_include_directories(include)
#				#ament_export_dependencies(EIGEN3)
#
#    else()
#        message(STATUS "Eigen not found...using ecl's internal eigen.")
#        set(ECL_EIGEN_IS_INTERNAL_3_1_2 1 CACHE BOOL "Using ecl's internal eigen implementation version 3.1.2")
#
#				ament_export_include_directories(include include/eigen3.1.2 include/eigen3.1.2/Eigen)
#				#ament_export_dependencies(EIGEN3)
#
#    endif()
#endif()

##############################################################################
# Configure header macros
##############################################################################

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/ecl/eigen.hpp.in ${CMAKE_CURRENT_SOURCE_DIR}/eigen.hpp)
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include FILES_MATCHING PATTERN "*.hpp")

##############################################################################
# Actual project configuration
##############################################################################

add_subdirectory(include)
add_subdirectory(license)


ament_package()
