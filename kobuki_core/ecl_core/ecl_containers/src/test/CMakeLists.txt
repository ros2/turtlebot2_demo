###############################################################################
# Google Tests Macro
###############################################################################

macro(ecl_containers_add_gtest test_name)
  ament_add_gtest(ecl_test_${test_name} ${test_name}.cpp)
  if(TARGET ecl_test_${test_name})
		ament_target_dependencies(ecl_test_${test_name} ecl_config
			ecl_errors
			ecl_exceptions
			ecl_formatters
			ecl_converters
			ecl_mpl
			ecl_type_traits
			ecl_utilities
		)

  endif()
endmacro()

###############################################################################
# Google Tests
###############################################################################

ecl_containers_add_gtest(arrays)
ecl_containers_add_gtest(arrays_mem_check)
ecl_containers_add_gtest(container_converters)
ecl_containers_add_gtest(stencil)
ecl_containers_add_gtest(fifo)
ecl_containers_add_gtest(push_and_pop)
