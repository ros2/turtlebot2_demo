set(Eigen_HEADERS AdolcForward BVH IterativeSolvers MatrixFunctions MoreVectorization AutoDiff AlignedVector3 Polynomials
                  FFT NonLinearOptimization SparseExtra IterativeSolvers
                  NumericalDiff Skyline MPRealSupport OpenGLSupport KroneckerProduct Splines
   )

install(FILES
  ${Eigen_HEADERS}
  DESTINATION include/unsupported/Eigen COMPONENT Devel
  )

add_subdirectory(src)
