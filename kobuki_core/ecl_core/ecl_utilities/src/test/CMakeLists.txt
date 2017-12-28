###############################################################################
# GTEST SETUP
###############################################################################

ament_add_gtest(test_function_objects function_objects.cpp)
ament_add_gtest(test_references references.cpp)
ament_add_gtest(test_singleton singleton.cpp)
ament_add_gtest(test_parameters parameters.cpp)
ament_add_gtest(test_flags flags.cpp)
