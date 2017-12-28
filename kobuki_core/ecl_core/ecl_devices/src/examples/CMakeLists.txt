###############################################################################
# Examples
###############################################################################

add_executable(demo_serial_timeouts serial_timeouts.cpp)
target_link_libraries(demo_serial_timeouts ${PROJECT_NAME})
ament_target_dependencies(demo_serial_timeouts
	ecl_config
	ecl_errors
	ecl_mpl
	ecl_type_traits
	ecl_utilities
	ecl_containers
	ecl_threads)


install(TARGETS demo_serial_timeouts RUNTIME DESTINATION bin)
