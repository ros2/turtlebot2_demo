/**
 * @file /ecl_math/src/test/simple.cpp
 *
 * @brief Tests the simple math implementations.
 *
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include "../../include/ecl/math/simple.hpp"

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(SimpleFunctionTests,sign) {
	EXPECT_EQ(ecl::sign(3),1);
	EXPECT_EQ(ecl::sign(0),0);
	EXPECT_EQ(ecl::sign(-2),-1);
	EXPECT_EQ(ecl::sign(3.3),1);
	EXPECT_EQ(ecl::sign(0.0),0);
	EXPECT_EQ(ecl::sign(-2.0),-1);
	EXPECT_EQ(ecl::psign(3),1);
	EXPECT_EQ(ecl::psign(0),1);
	EXPECT_EQ(ecl::psign(-2),-1);
	EXPECT_EQ(ecl::psign(3.3),1);
	EXPECT_EQ(ecl::psign(0.0),1);
	EXPECT_EQ(ecl::psign(-2.0),-1);
}

TEST(SimpleFunctionTests,cube_root) {
	EXPECT_FLOAT_EQ(2.0,ecl::cube_root(8.0));
	EXPECT_FLOAT_EQ(-2.0,ecl::cube_root(-8.0));
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}

