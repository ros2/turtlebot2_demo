FILE(GLOB Eigen_SuperLUSupport_SRCS "*.h")

INSTALL(FILES 
  ${Eigen_SuperLUSupport_SRCS}
  DESTINATION include/Eigen/src/SuperLUSupport COMPONENT Devel
  )
