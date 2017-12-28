FILE(GLOB Eigen_IterativeLinearSolvers_SRCS "*.h")

INSTALL(FILES
  ${Eigen_IterativeLinearSolvers_SRCS}
  DESTINATION include/Eigen/src/IterativeLinearSolvers COMPONENT Devel
  )
