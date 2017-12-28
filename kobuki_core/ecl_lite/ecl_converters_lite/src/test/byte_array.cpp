/**
 * @file src/test/byte_array.cpp
 *
 * @brief Unit test for the byte array functions.
 *
 * @date March 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include "../../include/ecl/converters_lite/byte_array.hpp"

/*****************************************************************************
** Globals
*****************************************************************************/

bool debug_output = true;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(ConverterTests,byteArrays) {
	ecl::int32 value;
	ecl::uint32 u_value;
	char three_six_three[4] = { 0x6b, 0x01, 0x00, 0x00 };
	unsigned char u_three_six_three[4] = { 0x6b, 0x01, 0x00, 0x00 };
	char minus_two[4] = { 0xfe, 0xff, 0xff, 0xff };
	unsigned char u_minus_two[4] = { 0xfe, 0xff, 0xff, 0xff };
	ecl::from_byte_array(value,three_six_three);
	if ( debug_output ) { std::cout << "value: " << value << std::endl; }
	EXPECT_EQ(363,value);
	ecl::from_byte_array(u_value,u_three_six_three);
	if ( debug_output ) { std::cout << "value: " << u_value << std::endl; }
	EXPECT_EQ(363,u_value);
	ecl::from_byte_array(value,minus_two);
	if ( debug_output ) { std::cout << "value: " << value << std::endl; }
	EXPECT_EQ(-2,value);
	ecl::from_byte_array(value,u_minus_two);
	if ( debug_output ) { std::cout << "value: " << value << std::endl; }
	EXPECT_EQ(-2,value);
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
