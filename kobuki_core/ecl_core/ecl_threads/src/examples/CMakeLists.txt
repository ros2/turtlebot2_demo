###############################################################################
# Files
###############################################################################

add_executable(demo_thread thread.cpp)
target_link_libraries(demo_thread ${PROJECT_NAME})
ament_target_dependencies(demo_thread ecl_config
	ecl_errors
	ecl_concepts
	ecl_exceptions
	ecl_time
	ecl_utilities)

install(TARGETS demo_thread RUNTIME DESTINATION bin)

add_executable(demo_mutex mutex.cpp)
target_link_libraries(demo_mutex ${PROJECT_NAME})
ament_target_dependencies(demo_mutex ecl_config
	ecl_errors
	ecl_concepts
	ecl_exceptions
	ecl_time
	ecl_utilities)

install(TARGETS demo_mutex RUNTIME DESTINATION bin)
