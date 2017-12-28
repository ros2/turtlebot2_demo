###############################################################################
# GTest setup
###############################################################################

macro(ecl_add_gtest test_name)
  ament_add_gtest(ecl_test_${test_name} ${test_name}.cpp)
  if(TARGET ecl_test_${test_name})
      target_link_libraries(ecl_test_${test_name} ${PROJECT_NAME})
  endif()
endmacro()

###############################################################################
# GTests
###############################################################################

ecl_add_gtest(covariance_ellipsoids)
ecl_add_gtest(cumulative_statistics)
