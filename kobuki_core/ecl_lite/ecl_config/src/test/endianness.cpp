/**
 * @file /ecl_config/src/test/endianness.cpp
 *
 * @brief Unit test for endianness.
 *
 * @date February, 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <gtest/gtest.h>
#include "../../include/ecl/config/endianness.hpp"

/*****************************************************************************
** Tests
*****************************************************************************/
/*
 * Doesn't matter what the result is, just provide coverage.
 * compile time if they aren't.
 */
TEST(TypeTests,fundamentals) {
	bool result = ecl::is_big_endian();
	result = false;
    SUCCEED();
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}


