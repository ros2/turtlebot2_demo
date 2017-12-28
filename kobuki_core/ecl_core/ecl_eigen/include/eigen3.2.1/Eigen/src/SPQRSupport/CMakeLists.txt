FILE(GLOB Eigen_SPQRSupport_SRCS "*.h")

INSTALL(FILES
  ${Eigen_SPQRSupport_SRCS}
  DESTINATION include/Eigen/src/SPQRSupport/ COMPONENT Devel
  )
