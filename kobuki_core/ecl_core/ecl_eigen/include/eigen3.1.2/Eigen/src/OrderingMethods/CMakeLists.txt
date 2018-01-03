FILE(GLOB Eigen_OrderingMethods_SRCS "*.h")

INSTALL(FILES
  ${Eigen_OrderingMethods_SRCS}
  DESTINATION include/Eigen/src/OrderingMethods COMPONENT Devel
  )
