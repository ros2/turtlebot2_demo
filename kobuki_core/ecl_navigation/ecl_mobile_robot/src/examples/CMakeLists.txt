###############################################################################
# Files
###############################################################################

add_executable(demo_partial_inverse partial_inverse.cpp)
target_link_libraries(demo_partial_inverse ${PROJECT_NAME})
ament_target_dependencies(demo_partial_inverse
	ecl_build
	ecl_errors
	ecl_geometry
	ecl_math
	ecl_formatters
	ecl_linear_algebra
)

install(TARGETS demo_partial_inverse RUNTIME DESTINATION lib/${PROJECT_NAME})
