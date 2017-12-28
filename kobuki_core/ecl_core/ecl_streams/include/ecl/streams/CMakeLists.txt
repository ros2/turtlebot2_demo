###############################################################################
# SUBDIRECTORIES
###############################################################################

add_subdirectory(manipulators)
add_subdirectory(text_streams)

###############################################################################
# FILES
###############################################################################

file(GLOB HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.hpp)

install(FILES ${HEADERS} DESTINATION include/ecl/streams)
