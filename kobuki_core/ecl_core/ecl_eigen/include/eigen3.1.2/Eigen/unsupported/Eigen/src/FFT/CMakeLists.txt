FILE(GLOB Eigen_FFT_SRCS "*.h")

INSTALL(FILES
  ${Eigen_FFT_SRCS}
  DESTINATION include/unsupported/Eigen/src/FFT COMPONENT Devel
  )
