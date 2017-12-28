###############################################################################
# GTEST SETUP
###############################################################################

add_executable(demo_formatters formatters.cpp)
ament_target_dependencies(${PROJECT_NAME} ecl_config ecl_errors)
target_link_libraries(demo_formatters ${PROJECT_NAME})
install(TARGETS demo_formatters RUNTIME DESTINATION lib/${PROJECT_NAME})
