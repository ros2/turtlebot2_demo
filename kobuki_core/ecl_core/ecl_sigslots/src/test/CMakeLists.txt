###############################################################################
# Test Macro
###############################################################################

macro(ecl_sigslots_add_gtest test_name)
  ament_add_gtest(ecl_test_${test_name} ${test_name}.cpp)
  if(TARGET ecl_test_${test_name})
		ament_target_dependencies(ecl_test_${test_name} ecl_config ecl_threads)
  endif()
endmacro()

###############################################################################
# Google Tests
###############################################################################

ecl_sigslots_add_gtest(sigslots sigslots.cpp)
