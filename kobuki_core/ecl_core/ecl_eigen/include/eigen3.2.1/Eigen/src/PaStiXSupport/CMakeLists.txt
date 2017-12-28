FILE(GLOB Eigen_PastixSupport_SRCS "*.h")

INSTALL(FILES
  ${Eigen_PastixSupport_SRCS}
  DESTINATION include/Eigen/src/PaStiXSupport COMPONENT Devel
  )
