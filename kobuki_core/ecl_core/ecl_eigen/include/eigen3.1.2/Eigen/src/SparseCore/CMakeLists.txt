FILE(GLOB Eigen_SparseCore_SRCS "*.h")

INSTALL(FILES 
  ${Eigen_SparseCore_SRCS}
  DESTINATION include/Eigen/src/SparseCore COMPONENT Devel
  )
