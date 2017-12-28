##############################################################################
# Files
##############################################################################

add_executable(demo_exceptions exceptions.cpp)
target_link_libraries(demo_exceptions ${PROJECT_NAME})
ament_target_dependencies(demo_exceptions ecl_config ecl_errors)
install(TARGETS demo_exceptions RUNTIME DESTINATION lib/${PROJECT_NAME})


add_executable(demo_exception_tracer exception_tracer.cpp)
target_link_libraries(demo_exceptions ${PROJECT_NAME})
ament_target_dependencies(demo_exception_tracer ecl_config ecl_errors)
install(TARGETS demo_exception_tracer RUNTIME DESTINATION lib/${PROJECT_NAME})
