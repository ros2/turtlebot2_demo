###############################################################################
# SOURCES
###############################################################################

# Note, macro magic stops us compiling objects for the wrong implementations below
SET(SOURCES
    cpuwatch_rt.cpp
    frequency.cpp
    sleep_win.cpp
    sleep_pos.cpp
    snooze_pos.cpp
    snooze_win.cpp
    stopwatch.cpp
    timestamp_base.cpp
    timestamp_pos.cpp
    timestamp_win.cpp
    time_data.cpp
)
###############################################################################
# External Library Dependencies
###############################################################################

#IF ( ECL_PLATFORM_HAS_POSIX_THREADS )
#  SET (LIBRARIES "-pthread ${LIBRARIES}")
#ENDIF ()

#IF(ECL_PLATFORM_HAS_RT_TIMERS)
#  SET (LIBRARIES "-lrt ${LIBRARIES}")
#ENDIF()

# Remove leading and trailing whitespace
#string(STRIP "${LIBRARIES}" LIBRARIES)

###############################################################################
# Install
###############################################################################

add_library(${PROJECT_NAME} SHARED ${SOURCES})
ament_target_dependencies(${PROJECT_NAME} ecl_config ecl_errors ecl_exceptions ecl_time_lite ecl_build)

install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION bin
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)
