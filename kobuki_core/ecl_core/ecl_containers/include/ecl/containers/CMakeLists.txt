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

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include FILES_MATCHING PATTERN "*.hpp")
