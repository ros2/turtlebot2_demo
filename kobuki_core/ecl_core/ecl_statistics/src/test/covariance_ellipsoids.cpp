/**
 * @file /src/test/covariance_ellipsoids.cpp
 *
 * @brief Unit Test for covariance ellipsoids.
 *
 * @date October 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <gtest/gtest.h>
#include <ecl/linear_algebra.hpp>
#include "../../include/ecl/statistics/covariance_ellipsoid.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::CovarianceEllipsoid2d;
using ecl::CovarianceEllipsoid3d;
using ecl::linear_algebra::Matrix2d;
using ecl::linear_algebra::Vector2d;
using ecl::linear_algebra::Matrix3d;
using ecl::linear_algebra::Vector3d;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(CovarianceTests,ellipsoid2D) {
    Matrix2d M;
	M << 3.0, 1.0, 1.0, 5.0; // must be a positive definite, symmetric matrix
//    M << 0.001801, -0.000047, -0.000047, 0.000259;
//    M << 0.0000009661, 0.0000065, 0.0000065, 0.0001261;

    CovarianceEllipsoid2d ellipse(M);
    const Vector2d& lengths = ellipse.lengths();
	double eigen_value_0 = lengths[0]*lengths[0];
    double angle = ellipse.rotation();
    const Vector2d& intercepts = ellipse.intercepts();
    const Matrix2d& axes = ellipse.axes();

	EXPECT_GT(2.328,lengths[0]); EXPECT_LT(2.326,lengths[0]);
    EXPECT_GT(5.415,eigen_value_0); EXPECT_LT(5.413,eigen_value_0);
    EXPECT_GT(1.179,angle); EXPECT_LT(1.177,angle);
	EXPECT_GT(1.674,intercepts[0]); EXPECT_LT(1.672,intercepts[0]);
	EXPECT_GT(0.3828,axes(0,0)); EXPECT_LT(0.3826,axes(0,0));

//    std::cout << std::endl;
//    std::cout << "***********************************************************" << std::endl;
//    std::cout << "                 Covariance Ellipsoid2d" << std::endl;
//    std::cout << "***********************************************************" << std::endl;
//    std::cout << std::endl;
//    std::cout << "Test Matrix: " << std::endl;
//    std::cout << std::endl;
//    std::cout << M << std::endl;
//    std::cout << std::endl;
//    std::cout << "Axis Lengths: " <<  ellipse.lengths().transpose() << std::endl;
//    std::cout << "Eigen Values: " << ellipse.lengths()(0)*ellipse.lengths()(0) << " " << ellipse.lengths()(1)*ellipse.lengths()(1) << std::endl;
//    std::cout << "Axis Vectors: " << std::endl;
//    std::cout << "Rotation angle: " << ellipse.rotation() << std::endl;
//    std::cout << "Intercepts: " << ellipse.intercepts().transpose() << std::endl;
//    std::cout << "Eigenvector Dot Product: " << ellipse.axes().block(0,0,2,1).transpose()*ellipse.axes().block(0,1,2,1) << std::endl;

}

TEST(CovarianceTests,ellipsoid3D) {
    Matrix3d P;
    double sigmaX(0.1);
    double sigmaY(0.5);
    double sigmaT(0.3);
    P << sigmaX*sigmaX, 0, 0, 0, sigmaY*sigmaY, 0, 0, 0, sigmaT*sigmaT;
	CovarianceEllipsoid3d ellipse(P);

    const Vector3d& lengths = ellipse.lengths();
	double eigen_value_0 = lengths[0]*lengths[0];
    const Matrix3d& axes = ellipse.axes();
    std::cout << axes << std::endl;
	EXPECT_GT(0.101,lengths[0]); EXPECT_LT(0.099,lengths[0]);
    EXPECT_GT(0.011,eigen_value_0); EXPECT_LT(0.009,eigen_value_0);
	EXPECT_GT(1.001,axes(0,0)); EXPECT_LT(0.999,axes(0,0));

//  std::cout << std::endl;
//	std::cout << "***********************************************************" << std::endl;
//	std::cout << "                 Covariance Ellipsoid3d" << std::endl;
//	std::cout << "***********************************************************" << std::endl;
//	std::cout << std::endl;
//	std::cout << "Suppose we handle robot pose variances on sigmaX, sigmaY," << std::endl;
//	std::cout << "sigmaT(heta)" << std::endl;
//	std::cout << std::endl;
//	std::cout << "[ sigmaX, sigmaY, sigmaT ]: [" << sigmaX << ", " << sigmaY << ", " << sigmaT << "]" << std::endl;
//	std::cout << P << std::endl;
//	std::cout << std::endl;
//	std::cout << "Now we build covariance matrix: " << std::endl;
//	std::cout << std::endl;
//	std::cout << "Axis Lengths: " <<  ellipse.lengths().transpose() << std::endl;
//	std::cout << "Eigen Values: " << ellipse.lengths()(0)*ellipse.lengths()(0)
//													<< " " << ellipse.lengths()(1)*ellipse.lengths()(1)
//													<< " " <<  ellipse.lengths()(2)*ellipse.lengths()(2)
//													<< std::endl;
//	std::cout << "Axis Vectors: " << std::endl;
//	std::cout << ellipse.axes() << std::endl;
//	std::cout << std::endl;

}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}


