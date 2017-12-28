FILE(GLOB Eigen_Splines_SRCS "*.h")

INSTALL(FILES
  ${Eigen_Splines_SRCS}
  DESTINATION include/unsupported/Eigen/src/Splines COMPONENT Devel
  )
