###############################################################################
# GTest setup
###############################################################################

macro(ecl_add_gtest TESTNAME)
  ament_add_gtest(ecl_test_${TESTNAME} ${TESTNAME}.cpp)
  if(TARGET ecl_test_${TESTNAME})
    target_link_libraries(ecl_test_${TESTNAME} ${PROJECT_NAME})
		ament_target_dependencies(ecl_test_${TESTNAME}
			ecl_build
			ecl_errors
			ecl_geometry
			ecl_math
			ecl_formatters
			ecl_linear_algebra
		)

  endif()
endmacro(ecl_add_gtest)

###############################################################################
# GTests
###############################################################################

ecl_add_gtest(partial_inverse)
