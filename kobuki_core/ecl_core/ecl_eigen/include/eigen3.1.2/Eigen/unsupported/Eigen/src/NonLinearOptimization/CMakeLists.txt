FILE(GLOB Eigen_NonLinearOptimization_SRCS "*.h")

INSTALL(FILES
  ${Eigen_NonLinearOptimization_SRCS}
  DESTINATION include/unsupported/Eigen/src/NonLinearOptimization COMPONENT Devel
  )
