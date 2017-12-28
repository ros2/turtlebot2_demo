FILE(GLOB Eigen_KroneckerProduct_SRCS "*.h")

INSTALL(FILES
  ${Eigen_KroneckerProduct_SRCS}
  DESTINATION include/unsupported/Eigen/src/KroneckerProduct COMPONENT Devel
  )
