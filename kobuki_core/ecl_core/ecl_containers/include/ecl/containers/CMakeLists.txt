###############################################################################
# SUBDIRECTORIES
###############################################################################

add_subdirectory(common)
add_subdirectory(array)
add_subdirectory(push_and_pop)
add_subdirectory(stencil)

###############################################################################
# FILES
###############################################################################

file(GLOB HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.hpp)

install(FILES ${HEADERS} DESTINATION include/ecl/containers)
