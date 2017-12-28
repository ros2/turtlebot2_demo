FILE(GLOB Eigen_PardisoSupport_SRCS "*.h")

INSTALL(FILES 
  ${Eigen_PardisoSupport_SRCS}
  DESTINATION include/Eigen/src/PardisoSupport COMPONENT Devel
  )
