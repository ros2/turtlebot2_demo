##############################################################################
# Files
##############################################################################

add_executable(ecl_detect_sizes detect_sizes.cpp)
set_target_properties(ecl_detect_sizes PROPERTIES RUNTIME_OUTPUT_DIRECTORY bin)
target_link_libraries(ecl_detect_sizes ${PROJECT_NAME})
install(TARGETS ecl_detect_sizes RUNTIME DESTINATION bin)
