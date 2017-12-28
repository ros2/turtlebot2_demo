###############################################################################
# Sources
###############################################################################

#file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

set(SOURCES
    detail/character_buffer.cpp
    detail/error_handler.cpp
    detail/exception_handler_pos.cpp
    #detail/socket_error_handler_pos.cpp
    detail/socket_exception_handler_pos.cpp
    console.cpp
    ofile_pos.cpp
    ofile_w32.cpp
    serial_pos.cpp # Don't need to split now as I put ifdef guards around the cpp's. Do to the others too!
    serial_w32.cpp
    shared_file.cpp
    socket_client_pos.cpp
    socket_server_pos.cpp
    string.cpp
    )

###############################################################################
# Install
###############################################################################

add_library(${PROJECT_NAME} SHARED ${SOURCES})
ament_target_dependencies(${PROJECT_NAME}
	ecl_config
	ecl_errors
	ecl_mpl
	ecl_type_traits
	ecl_utilities
	ecl_containers
	ecl_threads)

	install(TARGETS ${PROJECT_NAME}
	   RUNTIME DESTINATION bin
	   ARCHIVE DESTINATION lib
	   LIBRARY DESTINATION lib
	   PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
	)
