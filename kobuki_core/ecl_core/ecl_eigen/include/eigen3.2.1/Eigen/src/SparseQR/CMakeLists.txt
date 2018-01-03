FILE(GLOB Eigen_SparseQR_SRCS "*.h")

INSTALL(FILES
  ${Eigen_SparseQR_SRCS}
  DESTINATION include/Eigen/src/SparseQR/ COMPONENT Devel
  )
