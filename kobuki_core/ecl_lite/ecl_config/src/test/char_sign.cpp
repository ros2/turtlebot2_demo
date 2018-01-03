/**
 * @file /ecl_config/src/test/char_sign.cpp
 *
 * @brief Unit test for char sign check.
 *
 * @date February, 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <gtest/gtest.h>
#include "../../include/ecl/config/char_sign.hpp"

/*****************************************************************************
** Tests
*****************************************************************************/
/*
 * Doesn't matter what the result is, just provide coverage.
 * compile time if they aren't.
 */
TEST(TypeTests,fundamentals) {
	bool result = ecl::is_char_signed();
	if ( result ) {
		std::cout << "Char is signed." << std::endl;
	} else {
		std::cout << "Char is unsigned." << std::endl;
	}
	result = false; // stop gcc warnings.
    SUCCEED();
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
	std::cout << "CHAR_MIN" << CHAR_MIN << std::endl;
	testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}


