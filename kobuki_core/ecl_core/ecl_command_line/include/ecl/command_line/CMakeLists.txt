###############################################################################
# FILES
###############################################################################

FILE(GLOB HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.hpp)

INSTALL(FILES ${HEADERS} DESTINATION include/ecl/command_line)
