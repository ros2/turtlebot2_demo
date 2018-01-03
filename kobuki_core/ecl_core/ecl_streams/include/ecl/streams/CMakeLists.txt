###############################################################################
# SUBDIRECTORIES
###############################################################################

add_subdirectory(manipulators)
add_subdirectory(text_streams)

###############################################################################
# FILES
###############################################################################

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include FILES_MATCHING PATTERN "*.hpp")
