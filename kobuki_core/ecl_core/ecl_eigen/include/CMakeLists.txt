###############################################################################
# Directories
###############################################################################

#if(DEFINED ECL_EIGEN_IS_INTERNAL_3_1_2)
#    install(DIRECTORY eigen3.1.2/Eigen DESTINATION include)
#    install(DIRECTORY eigen3.1.2/Eigen/unsupported DESTINATION include)
#elseif(DEFINED ECL_EIGEN_IS_INTERNAL_3_2_1)
#    install(DIRECTORY eigen3.2.1/Eigen DESTINATION include)
		INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/eigen3.2.1/Eigen DESTINATION include)

    # don't like this, but seems to be the only way to get unsupported found in installspace
    #install(DIRECTORY eigen3.2.1/Eigen/unsupported DESTINATION include)
		INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/eigen3.2.1/Eigen/unsupported DESTINATION include)

#endif()
