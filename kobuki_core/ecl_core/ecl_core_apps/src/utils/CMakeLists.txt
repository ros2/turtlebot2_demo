###############################################################################
# Macro
###############################################################################

macro(ecl_add_utility name)
  add_executable(ecl_${name} ${name}.cpp)
	ament_target_dependencies(ecl_${name}
	ecl_build
  ecl_license
  ecl_config
  ecl_linear_algebra
  ecl_command_line
  ecl_converters
  ecl_containers
  ecl_devices
  ecl_errors
  ecl_exceptions
  ecl_formatters
  ecl_geometry
  ecl_ipc
  ecl_sigslots
  ecl_streams
  ecl_threads
  ecl_type_traits
  ecl_time_lite
	)

  install(TARGETS ecl_${name} RUNTIME DESTINATION bin)
endmacro()

macro(ecl_add_global_utility name)
  add_executable(ecl_${name} ${name}.cpp)
	ament_target_dependencies(ecl_${name}
	ecl_build
  ecl_license
  ecl_config
  ecl_linear_algebra
  ecl_command_line
  ecl_converters
  ecl_containers
  ecl_devices
  ecl_errors
  ecl_exceptions
  ecl_formatters
  ecl_geometry
  ecl_ipc
  ecl_sigslots
  ecl_streams
  ecl_threads
  ecl_type_traits
  ecl_time_lite
	)
	set_target_properties(ecl_${name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY bin)
  install(TARGETS ecl_${name} RUNTIME DESTINATION bin)
endmacro()

###############################################################################
# Setup
###############################################################################

ecl_add_utility(process_statistics)
ecl_add_utility(hex)
ecl_add_utility(serial)
ecl_add_utility(socket_client)
ecl_add_utility(socket_server)
ecl_add_global_utility(yaw2quaternion)
ecl_add_global_utility(quaternion2yaw)
