###############################################################################
# Building ecl packages (c.f. rosbuild.cmake for ros)
###############################################################################
#
# NOT FOR REGULAR CONSUMPTION! THIS IS ONLY USED BY ECL PACKAGES!
#

#ifdef DOXYGEN_SHOULD_SKIP_THIS

###############################################################################
# Source Installer
###############################################################################
#
# Creates custom targets for collecting all *.c, *.cpp, *.h, *.hpp files 
# together and installs them to CMAKE_INSTALL_PREFIX under the 
# include/pkg_name and src/pkg_name directories respectively.
#
# This is useful for collecting sources for firmware builds in another ide.
#
macro(ecl_roll_source_installs)
  FILE(GLOB_RECURSE PACKAGE_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.h *.hpp)
  FILE(GLOB_RECURSE PACKAGE_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/lib/*.c src/lib/*.cpp)
  add_custom_target(sources
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      COMMENT "Installing sources."
    )
  foreach(_file ${PACKAGE_HEADERS})
    get_filename_component(_dir ${_file} PATH)
    get_filename_component(_filename ${_file} NAME)
    string(REGEX REPLACE "/" "_" _target_name ${_file})
    add_custom_target(include_${_target_name}
      ${CMAKE_COMMAND} -E make_directory ${CMAKE_INSTALL_PREFIX}/${_dir}
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_file} ${CMAKE_INSTALL_PREFIX}/${_file}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    add_dependencies(sources include_${_target_name})
  endforeach(_file ${PACKAGE_HEADERS})
  foreach(_file ${PACKAGE_SOURCES})
    set(_dir ${CMAKE_INSTALL_PREFIX}/src/${PROJECT_NAME})
    get_filename_component(_filename ${_file} NAME)
    string(REGEX REPLACE "/" "_" _target_name ${_file})
    # Should probably convert slashes to underscores instead here.
    add_custom_target(source_${_target_name}
      ${CMAKE_COMMAND} -E make_directory ${_dir}
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_file} ${_dir}/${_filename}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    add_dependencies(sources source_${_target_name})
  endforeach(_file ${PACKAGE_SOURCES})
endmacro()

#endif DOXYGEN_SHOULD_SKIP_THIS

