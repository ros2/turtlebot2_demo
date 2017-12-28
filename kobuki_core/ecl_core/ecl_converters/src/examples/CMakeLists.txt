##############################################################################
# Files
##############################################################################

add_executable(demo_float_converters float_converters.cpp)
ament_target_dependencies(demo_float_converters ecl_exceptions)

install(TARGETS demo_float_converters RUNTIME DESTINATION lib)
