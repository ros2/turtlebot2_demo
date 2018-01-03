###############################################################################
# GENERATE HEADERS
#
# The cmake/project_generate_headers.cmake makes sure you have ecl.hpp built
# first.
###############################################################################

###################################
# Platform
###################################

IF(APPLE)
  SET(ECL_IS_APPLE 1)
ENDIF()
IF(PLATFORM_IS_POSIX)
  SET(ECL_IS_POSIX 1)
ENDIF()
IF(WIN32)
  SET(ECL_IS_WIN32 1)
ENDIF()

###################################
# Threads
###################################

IF( ECL_PLATFORM_HAS_POSIX_THREADS )
  SET(ECL_HAS_POSIX_THREADS 1)
ELSEIF( ECL_PLATFORM_HAS_WIN32_THREADS )
  SET(ECL_HAS_WIN32_THREADS 1)
ENDIF()

###################################
# TYPES
###################################

SET(ECL_SIZE_OF_CHAR ${PLATFORM_SIZE_OF_CHAR})
SET(ECL_SIZE_OF_SHORT ${PLATFORM_SIZE_OF_SHORT})
SET(ECL_SIZE_OF_INT ${PLATFORM_SIZE_OF_INT})
SET(ECL_SIZE_OF_LONG ${PLATFORM_SIZE_OF_LONG})
SET(ECL_SIZE_OF_LONG_LONG ${PLATFORM_SIZE_OF_LONG_LONG})
SET(ECL_SIZE_OF_FLOAT ${PLATFORM_SIZE_OF_FLOAT})
SET(ECL_SIZE_OF_DOUBLE ${PLATFORM_SIZE_OF_DOUBLE})
SET(ECL_SIZE_OF_LONG_DOUBLE ${PLATFORM_SIZE_OF_LONG_DOUBLE})
IF( PLATFORM_IS_32_BIT )
  SET(ECL_IS_32_BIT 1)
ELSEIF( PLATFORM_IS_64_BIT )
  SET(ECL_IS_64_BIT 1)
ENDIF( PLATFORM_IS_32_BIT )

###################################
# Library type
###################################
if(BUILD_SHARED_LIBS)
  set(ECL_HAS_SHARED_LIBS 1)
else()
  set(ECL_HAS_STATIC_LIBS 1)
endif()

###############################################################################
# GENERATE HEADER
###############################################################################

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ecl.hpp.in ${CMAKE_CURRENT_SOURCE_DIR}/ecl.hpp)

###############################################################################
# FILES
###############################################################################

file(GLOB HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.hpp)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include FILES_MATCHING PATTERN "*.hpp")


# If we want to supply the version number in major/minor/patch
# split version in parts and pass to extra file
#string(REPLACE "." ";" roscpp_VERSION_LIST "${roscpp_VERSION}")
#list(LENGTH roscpp_VERSION_LIST _count)
#if(NOT _count EQUAL 3)
#  message(FATAL_ERROR "roscpp version '${roscpp_VERSION}' does not match 'MAJOR.MINOR.PATCH' pattern")
#endif()
#list(GET roscpp_VERSION_LIST 0 roscpp_VERSION_MAJOR)
#list(GET roscpp_VERSION_LIST 1 roscpp_VERSION_MINOR)
#list(GET roscpp_VERSION_LIST 2 roscpp_VERSION_PATCH)
