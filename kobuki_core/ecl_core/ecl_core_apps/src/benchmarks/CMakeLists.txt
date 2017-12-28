###############################################################################
# Macro
###############################################################################

macro(ecl_add_benchmark name)
  add_executable(bench_${name} ${name}.cpp)
	ament_target_dependencies(bench_${name}
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
  install(TARGETS bench_${name} RUNTIME DESTINATION bin)
endmacro()

###############################################################################
# Setup
###############################################################################

ecl_add_benchmark(containers)
ecl_add_benchmark(files)
ecl_add_benchmark(flops)
ecl_add_benchmark(exceptions)
ecl_add_benchmark(snooze)
ecl_add_benchmark(streams)
ecl_add_benchmark(string_conversions)

# Sparse is still unstable...and got modified in quantal, comment out for now.
#ecl_add_benchmark(eigen_sparse)
ecl_add_benchmark(eigen3_inverse)
ecl_add_benchmark(eigen3_decompositions)
ecl_add_benchmark(eigen3_transforms)
