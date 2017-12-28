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

file(GLOB HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.hpp)

install(FILES ${HEADERS} DESTINATION include/ecl/filesystem)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/config.hpp DESTINATION include/ecl/filesystem)
