###############################################################################
# SETUP
###############################################################################

add_executable(ecl_detect_endian detect_endian.cpp)
set_target_properties(ecl_detect_endian PROPERTIES RUNTIME_OUTPUT_DIRECTORY bin)
install(TARGETS ecl_detect_endian RUNTIME DESTINATION bin)

if(NOT MSVC)
  add_executable(ecl_detect_posix detect_posix.cpp)
  set_target_properties(ecl_detect_posix PROPERTIES RUNTIME_OUTPUT_DIRECTORY bin)
  install(TARGETS ecl_detect_posix RUNTIME DESTINATION bin)
endif()
