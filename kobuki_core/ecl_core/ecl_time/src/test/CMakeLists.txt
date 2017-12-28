###############################################################################
# Test Macro
###############################################################################

macro(ecl_add_gtest test_name)
  ament_add_gtest(ecl_test_${test_name} ${test_name}.cpp)
  # If we don't have gtest installed, then the target doesn't get made - check!
  if(TARGET ecl_test_${test_name})
      target_link_libraries(ecl_test_${test_name} ${PROJECT_NAME})
  endif()
endmacro()

###############################################################################
# Google Tests
###############################################################################

ecl_add_gtest(cpuwatch_rt)
ecl_add_gtest(frequency)
ecl_add_gtest(sleep)
ecl_add_gtest(snooze)
ecl_add_gtest(timestamp)
ecl_add_gtest(time_data)
ecl_add_gtest(stopwatch)
