/**
 * @file /ecl_math/src/test/fuzzy.cpp
 *
 * @brief Tests the fuzzy math implementations.
 *
 * @date Dec 30, 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include "../../include/ecl/math/fuzzy.hpp"

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(FuzzyTests,isApprox) {
	EXPECT_TRUE(ecl::isApprox(3.0,3.0000000000000001));
	EXPECT_FALSE(ecl::isApprox(3.0,3.1000000000000001));
	EXPECT_TRUE(ecl::isApprox(3,3.0000000000000001));
	EXPECT_FALSE(ecl::isApprox(3,3.1000000000000001));
	EXPECT_TRUE(ecl::isApprox(3.0000000000000001,3));
	EXPECT_FALSE(ecl::isApprox(3.1000000000000001,3));
}

TEST(FuzzyTests,isApproxOrLessThan) {
	EXPECT_TRUE(ecl::isApproxOrLessThan(3.0,3.0000000000000001));
	EXPECT_TRUE(ecl::isApproxOrLessThan(2.0,3.0000000000000001));
	EXPECT_FALSE(ecl::isApproxOrLessThan(4.0,3.0000000000000001));
	EXPECT_TRUE(ecl::isApproxOrLessThan(3,3.0000000000000001));
	EXPECT_TRUE(ecl::isApproxOrLessThan(2,3.0000000000000001));
	EXPECT_FALSE(ecl::isApproxOrLessThan(4,3.0000000000000001));
	EXPECT_TRUE(ecl::isApproxOrLessThan(3.0000000000000001,3));
	EXPECT_FALSE(ecl::isApproxOrLessThan(3.0000000000000001,2));
	EXPECT_TRUE(ecl::isApproxOrLessThan(3.0000000000000001,4));
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}

