##############################################################################
# Files
##############################################################################

add_executable(demo_sigslots_manager sigslots_manager.cpp)
ament_target_dependencies(demo_sigslots_manager ecl_config ecl_threads)
install(TARGETS demo_sigslots_manager RUNTIME DESTINATION bin)

add_executable(demo_sigslots sigslots.cpp)
ament_target_dependencies(demo_sigslots ecl_config ecl_threads)
install(TARGETS demo_sigslots RUNTIME DESTINATION bin)
