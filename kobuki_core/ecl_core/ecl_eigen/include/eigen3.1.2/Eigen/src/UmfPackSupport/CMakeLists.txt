FILE(GLOB Eigen_UmfPackSupport_SRCS "*.h")

INSTALL(FILES 
  ${Eigen_UmfPackSupport_SRCS}
  DESTINATION include/Eigen/src/UmfPackSupport COMPONENT Devel
  )
