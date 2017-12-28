FILE(GLOB Eigen_Skyline_SRCS "*.h")

INSTALL(FILES
  ${Eigen_Skyline_SRCS}
  DESTINATION include/unsupported/Eigen/src/Skyline COMPONENT Devel
  )
