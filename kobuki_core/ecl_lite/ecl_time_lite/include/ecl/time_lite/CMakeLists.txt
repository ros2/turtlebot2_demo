###############################################################################
# Generate Header
###############################################################################
##############################################################################
# Initialisation
##############################################################################


##############################################################################
# Symbol Generation (for ecl/time_lite/config.hpp
##############################################################################

# see ecl_detect_posix and ecl_detect_timers in
# ecl_build/cmake/modules/ecl_platform_detection.cmake

if(ECL_POSIX_HAS_CLOCK_GETTIME)
  set(ECL_PRIVATE_HAS_CLOCK_GETTIME 1)
endif()
if(ECL_POSIX_HAS_CLOCK_NANOSLEEP)
  set(ECL_PRIVATE_HAS_CLOCK_NANOSLEEP 1)
endif()
if(ECL_POSIX_HAS_CLOCK_MONOTONIC)
  set(ECL_HAS_CLOCK_MONOTONIC 1)
endif()
if(ECL_POSIX_HAS_CPUTIME)
  set(ECL_HAS_CPUTIME 1)
endif()
if(ECL_PLATFORM_HAS_MACH_TIMERS)
  set(ECL_HAS_MACH_TIMERS 1)
endif()
if(ECL_PLATFORM_HAS_WIN_TIMERS)
  set(ECL_HAS_WIN_TIMERS 1)
endif()
if(ECL_PLATFORM_HAS_RT_TIMERS)
  set(ECL_HAS_RT_TIMERS 1)
endif()
if(ECL_PLATFORM_HAS_POSIX_TIMERS)
  set(ECL_HAS_POSIX_TIMERS 1)
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.hpp.in ${CMAKE_CURRENT_SOURCE_DIR}/config.hpp)

##############################################################################
# Files
##############################################################################

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include FILES_MATCHING PATTERN "*.hpp")
