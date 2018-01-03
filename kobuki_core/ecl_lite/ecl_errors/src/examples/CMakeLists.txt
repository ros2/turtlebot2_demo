###############################################################################
# Examples
###############################################################################

add_executable(ecl_demo_errors errors.cpp)
target_link_libraries(ecl_demo_errors ecl_errors)
install(TARGETS ecl_demo_errors RUNTIME DESTINATION bin) 
