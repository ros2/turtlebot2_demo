FILE(GLOB Eigen_CholmodSupport_SRCS "*.h")

INSTALL(FILES 
  ${Eigen_CholmodSupport_SRCS}
  DESTINATION include/Eigen/src/CholmodSupport COMPONENT Devel
  )
