###############################################################################
# Test Macro
###############################################################################

macro(ecl_type_traits_add_gtest test_name)
  ament_add_gtest(test_${test_name} ${test_name}.cpp)
  # If we don't have gtest installed, then the target doesn't get made - check!
  if(TARGET test_${test_name})
      target_link_libraries(test_${test_name} ${PROJECT_NAME})
  endif()
endmacro()

###############################################################################
# Google Tests
###############################################################################

##############################################################################
# Files
##############################################################################

ecl_type_traits_add_gtest(numeric_limits)
ecl_type_traits_add_gtest(fundamental_types)
