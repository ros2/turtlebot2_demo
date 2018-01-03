FILE(GLOB Eigen_SparseCholesky_SRCS "*.h")

INSTALL(FILES
  ${Eigen_SparseCholesky_SRCS}
  DESTINATION include/Eigen/src/SparseCholesky COMPONENT Devel
  )
