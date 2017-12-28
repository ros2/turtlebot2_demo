###############################################################################
# Build utilities
###############################################################################

#ifdef DOXYGEN_SHOULD_SKIP_THIS

###############################
# Download
###############################
# Adds a custom command for downloading a url to a 
# particular file.
#
# Depends : -
# Outputs : ${file}
#
# Note: ${file} can be 
# - relative in which case it is rel to ${CMAKE_BINARY_DIR}
# - absolute (e.g. ${CMAKE_BINARY_DIR}/dude.tar.gz
#
# Example:
#
#   URL=http://snorriheim.dnsdojo.com/tmp/${TARBALL}
#   TARBALL=dude.tar.gz
#   ecl_download(${URL} ${TARBALL})
#
macro(ecl_download url file)
    add_custom_command(OUTPUT ${file}
        COMMAND wget ${url} -O ${file}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Downloading ${url}->${file}."
        )
endmacro()

###############################
# Extract
###############################
# Adds a custom command for untar'ing a tarball in the specified dir.
#
# Depends : ${tarball}
# Outputs : ${dir}/extracted
# 
# Example:
# 
#   TARBALL=${CMAKE_BINARY_DIR}/dude.tar.gz
#   URL=http://snorriheim.dnsdojo.com/tmp/${TARBALL}
#   ecl_download(${TARBALL} ${URL})
#   ecl_extract_tarball(${TARBALL} ${CMAKE_BINARY_DIR}/fakeroot) 
#
macro(ecl_extract_tarball tarball dir)
    add_custom_command(OUTPUT ${dir}/extracted
        COMMAND mkdir -p ${dir}
        COMMAND tar -xvzf ${tarball} -C ${dir}
        COMMAND touch ${dir}/extracted
        DEPENDS ${tarball}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Extracting ${tarball} -> ${dir}."
        VERBATIM
        )
endmacro()

# Similar to the untarball command, but for bzips.
#
macro(ecl_extract_bzip2 bzip2 dir)
    add_custom_command(OUTPUT ${dir}/extracted
        COMMAND tar -xvjf ${bzip2} -C ${dir}
        COMMAND touch ${dir}/extracted
        DEPENDS ${bzip2}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Extracting ${bzip2} -> ${dir}."
        VERBATIM
        )
endmacro()

###############################
# Autotools Compile
###############################
# This adds a custom command for a typical autotools compile.
# Be sure to set up the configure command correctly before calling.
#
# Depends: ${depends} (input argument)
# Outputs: ${dir}/compiled
#
macro(ecl_autotools_compile configure_command dir depends)
    add_custom_command(OUTPUT ${dir}/compiled
        COMMAND ${configure_command}
        COMMAND make $ENV{ROS_PARALLEL_JOBS} # If not ros, then that is just empty anyway
        COMMAND make install
        COMMAND touch ${dir}/compiled
        DEPENDS ${depends}
        WORKING_DIRECTORY ${dir}
        COMMENT "Compiling ${dir}."
        VERBATIM
        )
endmacro()

###############################
# Uninstall
###############################
# Creates an uninstall target.
# 
# To do this, it needs to find the uninstall template and prep it.
# In ros, we can grab it from ecl_build, otherwise we use pkgconfig
# to find it from an already installed ecl.
macro(ecl_add_uninstall_target)

    # Find the template
    if(ROSBUILD_init_called)
        rosbuild_find_ros_package(ecl_build)
        set(ECL_CMAKE_TEMPLATES_PATH ${ecl_build_PACKAGE_PATH}/cmake/templates)
    else()
        find_package(PkgConfig)
        if(PKG_CONFIG_FOUND)
            PKG_CHECK_MODULES(ECL_CMAKE ecl_cmake)
            set(ECL_CMAKE_TEMPLATES_PATH ${ECL_CMAKE_PREFIX}/templates)
        else()
            message(FATAL_ERROR "Ecl cmake modules path not found (missing pkgconfig?).")
        endif()
    endif()
    
    # Prep it
    configure_file(
        "${ECL_CMAKE_TEMPLATES_PATH}/uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake"
        IMMEDIATE @ONLY)
        
    # Add the target
    add_custom_target(uninstall
        "${CMAKE_COMMAND}" -P
        "${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake")
endmacro()

#endif DOXYGEN_SHOULD_SKIP_THIS
