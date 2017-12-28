##############################################################################
# Files
##############################################################################

add_executable(demo_time_functions time_functions.cpp)
target_link_libraries(demo_time_functions ecl_time_lite)
install(TARGETS demo_time_functions RUNTIME DESTINATION bin)
