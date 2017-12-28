###############################################################################
# SUBDIRECTORIES
###############################################################################

add_subdirectory(eigen)
add_subdirectory(sophus)

###############################################################################
# FILES
###############################################################################

file(GLOB HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.hpp)

install(FILES ${HEADERS} DESTINATION include/ecl/linear_algebra)
