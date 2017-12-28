FILE(GLOB Eigen_SparseLU_SRCS "*.h")

INSTALL(FILES
  ${Eigen_SparseLU_SRCS}
  DESTINATION include/Eigen/src/SparseLU COMPONENT Devel
  )
