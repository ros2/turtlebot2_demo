##############################################################################
# Files
##############################################################################

add_executable(demo_command_line command_line.cpp)
install(TARGETS demo_command_line RUNTIME DESTINATION lib/${PROJECT_NAME})
