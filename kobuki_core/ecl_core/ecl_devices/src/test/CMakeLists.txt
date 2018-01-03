###############################################################################
# Macro
###############################################################################

macro(ecl_devices_add_gtest test_name)
  ament_add_gtest(ecl_test_${test_name} ${test_name}.cpp)
  if(TARGET ecl_test_${test_name})
		ament_target_dependencies(ecl_test_${test_name} ecl_config
			ecl_errors
			ecl_mpl
			ecl_type_traits
			ecl_utilities
			ecl_containers
			ecl_threads)
  endif()
endmacro()

###############################################################################
# Google Tests
###############################################################################

ecl_devices_add_gtest(shared_files)
ecl_devices_add_gtest(files)
