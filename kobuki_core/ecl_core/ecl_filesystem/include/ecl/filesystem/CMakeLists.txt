##############################################################################
# Symbol Generation (for ecl/filesystem/config.hpp
##############################################################################

# see ecl_detect_posix and ecl_detect_filesystem in

if(ECL_POSIX_HAS_REALPATH)
  set(ECL_PRIVATE_HAS_POSIX_REALPATH 1)
endif()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.hpp.in ${CMAKE_CURRENT_SOURCE_DIR}/config.hpp)

##############################################################################
# Files
##############################################################################

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include FILES_MATCHING PATTERN "*.hpp")
