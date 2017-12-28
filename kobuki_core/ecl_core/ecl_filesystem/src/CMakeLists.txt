###############################################################################
# SUBDIRECTORIES
###############################################################################

add_subdirectory(lib)
if(AMENT_ENABLE_TESTING)
  add_subdirectory(test)
endif()
