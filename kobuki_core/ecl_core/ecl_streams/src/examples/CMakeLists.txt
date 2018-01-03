##############################################################################
# Files
##############################################################################

add_executable(demo_console_streams console_streams.cpp)
target_link_libraries(demo_console_streams ${PROJECT_NAME})
ament_target_dependencies(demo_console_streams
	ecl_errors
	ecl_concepts
	ecl_devices
	ecl_time
	ecl_converters
	ecl_type_traits
)

install(TARGETS demo_console_streams RUNTIME DESTINATION bin)

add_executable(demo_log_streams log_streams.cpp)
target_link_libraries(demo_log_streams ${PROJECT_NAME})
ament_target_dependencies(demo_log_streams
	ecl_errors
	ecl_concepts
	ecl_devices
	ecl_time
	ecl_converters
	ecl_type_traits
)
install(TARGETS demo_log_streams RUNTIME DESTINATION bin)
