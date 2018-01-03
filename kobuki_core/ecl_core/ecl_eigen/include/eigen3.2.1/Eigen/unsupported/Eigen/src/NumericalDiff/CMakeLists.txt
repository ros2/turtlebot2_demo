FILE(GLOB Eigen_NumericalDiff_SRCS "*.h")

INSTALL(FILES
  ${Eigen_NumericalDiff_SRCS}
  DESTINATION include/unsupported/Eigen/src/NumericalDiff COMPONENT Devel
  )
