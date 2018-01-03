###############################################################################
# Sources
###############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

###############################################################################
# External Library Dependencies
###############################################################################

if(ECL_PLATFORM_HAS_POSIX_THREADS)
  set (thread_LIBRARIES "-pthread")
endif()

# Remove leading and trailing whitespace
string(STRIP "${LIBRARIES}" LIBRARIES)

###############################################################################
# Install
###############################################################################

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${thread_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} ecl_config
	ecl_errors
	ecl_concepts
	ecl_exceptions
	ecl_time
	ecl_utilities)

	install(TARGETS ${PROJECT_NAME}
	   RUNTIME DESTINATION bin
	   ARCHIVE DESTINATION lib
	   LIBRARY DESTINATION lib
	   #PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
	)
