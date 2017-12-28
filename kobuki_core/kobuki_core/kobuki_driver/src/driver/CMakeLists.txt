##############################################################################
# SOURCES
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# CONFIGURATIONS
##############################################################################

set(VERSION_FILE ${PROJECT_SOURCE_DIR}/build/version_info.cpp)
configure_file(version_info.cpp.in ${VERSION_FILE} @ONLY)

##############################################################################
# LIBRARIES
##############################################################################

add_library(kobuki SHARED ${SOURCES} ${VERSION_FILE})
target_link_libraries(kobuki ${catkin_LIBRARIES})
ament_target_dependencies(kobuki
	ecl_command_line
	ecl_converters
	ecl_devices
	ecl_geometry
	ecl_mobile_robot
	ecl_sigslots
	ecl_time
)
install(TARGETS kobuki
        DESTINATION lib
)
