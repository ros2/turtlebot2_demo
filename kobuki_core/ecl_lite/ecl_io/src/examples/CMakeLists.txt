##############################################################################
# Macros
##############################################################################

macro(ecl_add_io_example exe)
  add_executable(ecl_demo_${exe} ${ARGN})
  target_link_libraries(ecl_demo_${exe} ${PROJECT_NAME} ${sockets_LIBRARIES})
  install(TARGETS ecl_demo_${exe} RUNTIME DESTINATION bin)
endmacro()

##############################################################################
# Examples
##############################################################################

ecl_add_io_example(sockets sockets.cpp)
ecl_add_io_example(socketpair socketpair.cpp)
ecl_add_io_example(poll poll.cpp)
