/**
 * @file /src/test/norms.cpp
 *
 * @brief Unit Test for the norms.
 *
 * @date April 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <gtest/gtest.h>
#include "../../include/ecl/math/norms.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::EuclideanNorm;
using ecl::euclidean_norm;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(NormTests,euclideanNorm) {
	EXPECT_EQ(5,EuclideanNorm()(3,4));
}

TEST(NormTests,euclideanNormFloats) {
	float x = 3.0;
	float y = 4.0;
	float z = 5.0;
	EXPECT_EQ(z,EuclideanNorm()(x,y));
}

TEST(NormTests,euclideanNormDoubles) {
	double x = 3.0;
	double y = 4.0;
	double z = 5.0;
	EXPECT_EQ(z,EuclideanNorm()(x,y));
}

TEST(NormTests,euclideanNormFunctions) {
	double x = 3.0;
	double y = 4.0;
	double z = 5.0;
	EXPECT_EQ(z,euclidean_norm(x,y));
	float xf = 3.0;
	float yf = 4.0;
	float zf = 5.0;
	EXPECT_EQ(zf,euclidean_norm(xf,yf));
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {
	std::cout << "Numeric Limits<float>: " << std::numeric_limits<float>::min() << std::endl;
	std::cout << "Numeric Limits<float>: " << std::numeric_limits<float>::max() << std::endl;
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}

