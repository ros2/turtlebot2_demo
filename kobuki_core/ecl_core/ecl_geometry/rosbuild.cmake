if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/package.cmake)
  include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)
endif()

add_subdirectory(include)
add_subdirectory(src/lib)
add_subdirectory(pkg-config)
