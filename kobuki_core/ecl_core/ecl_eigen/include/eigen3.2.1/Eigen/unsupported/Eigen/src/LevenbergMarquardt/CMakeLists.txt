FILE(GLOB Eigen_LevenbergMarquardt_SRCS "*.h")

INSTALL(FILES
  ${Eigen_LevenbergMarquardt_SRCS}
  DESTINATION include/Eigen/src/LevenbergMarquardt COMPONENT Devel
  )
