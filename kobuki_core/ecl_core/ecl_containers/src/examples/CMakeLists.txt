###############################################################################
# Examples
###############################################################################

add_executable(demo_container_formatters formatters.cpp)
ament_target_dependencies(demo_container_formatters ecl_config
	ecl_errors
	ecl_exceptions
	ecl_formatters
	ecl_converters
	ecl_mpl
	ecl_type_traits
	ecl_utilities
)

install(TARGETS demo_container_formatters RUNTIME DESTINATION bin)
