FILE(GLOB Eigen_MetisSupport_SRCS "*.h")

INSTALL(FILES
  ${Eigen_MetisSupport_SRCS}
  DESTINATION include/Eigen/src/MetisSupport COMPONENT Devel
  )
