###############################################################################
# GTEST SETUP
###############################################################################


macro(ecl_streams_add_gtest test_name)
  ament_add_gtest(ecl_test_${test_name} ${test_name}.cpp)
  # If we don't have gtest installed, then the target doesn't get made - check!
  if(TARGET ecl_test_${test_name})
			ament_target_dependencies(ecl_test_${test_name}
				ecl_errors
				ecl_concepts
				ecl_devices
				ecl_time
				ecl_converters
				ecl_type_traits
			)
  endif()
endmacro()

ecl_streams_add_gtest(file_streams)
ecl_streams_add_gtest(string_streams)
