/**
 * @file /src/test/converters.cpp
 *
 * @brief Unit Test for @ref ecl::Converter "Converter".
 *
 * @date April 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <string>
#include <gtest/gtest.h>
#include "../../include/ecl/converters/to_byte_array.hpp"
#include "../../include/ecl/converters/char.hpp"
#include "../../include/ecl/converters/char_strings.hpp"
#include "../../include/ecl/converters/string.hpp"
#include "../../include/ecl/converters/integers.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using std::string;
using ecl::Converter;

/*****************************************************************************
** Globals
*****************************************************************************/

const bool debug_output = true;
//const bool debug_output = false;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(Converter,toChar) {
    Converter<char,unsigned int> uintToAscii;
    Converter<char,int> intToAscii;
    Converter<char> toAscii;

    if(debug_output) {
    	std::cout << "uintToAscii(8U): " << uintToAscii(8U)  << std::endl;
    	std::cout << "intToAscii(8): " << intToAscii(8)  << std::endl;
    	std::cout << "toAscii(8): " << toAscii(8)  << std::endl;
    	std::cout << "toAscii(8U): " << toAscii(8U)  << std::endl;
    }

    EXPECT_EQ('8',uintToAscii(8U));
    EXPECT_EQ('8',intToAscii(8));
    EXPECT_EQ('8',toAscii(8U));
    EXPECT_EQ('8',toAscii(8));
}

TEST(Converter,toString) {
    Converter<string,int> intToString;
    if(debug_output) {
    	std::cout << "intToString(123): " << intToString(123) << std::endl;
    	std::cout << "intToString(-123): " << intToString(-123) << std::endl;
    }
    EXPECT_EQ(string("123"),intToString(123));
    EXPECT_EQ(string("-123"),intToString(-123));
}

TEST(Converter,toInt) {
    Converter<int,void> toInt;
    if(debug_output) {
		std::cout << "toInt:" << std::endl;
		std::cout << "  string -> " << toInt(string("123")) << std::endl;
//		std::cout << "  char*  -> " << toInt("-123") << std::endl;
		std::cout << "  char   -> " << toInt('3') << std::endl;
    }
    EXPECT_EQ(123,toInt(string("123")));
//    EXPECT_EQ(-123,toInt("-123")); // fix this once i have the proper byte array converter in.
    EXPECT_EQ(3,toInt(char('3')));
}

TEST(Converter,toCharString) {
    // Bit hard to test these.
    Converter<char*> toCharString;
    char c = 'A';
    unsigned char uc = 'A';
    short s = -111;
    unsigned short us = 111;
    int i = -111;
    unsigned int ui = 111;
    long l = -111;
    unsigned long ul = 111;
    long long ll = -111111;
    unsigned long long ull = 11111111;
    float f = -321.23;
    double d = -321.23;
    char buffer[44];
    Converter<char*,float> floatToCharString(buffer, buffer+4);

    if(debug_output) {
    	std::cout << "toCharString:" << std::endl;
		std::cout << "  char          -> " << toCharString(c) << std::endl;
		std::cout << "  unsigned char -> " << toCharString(uc) << std::endl;
		std::cout << "  short         -> " << toCharString(s) << std::endl;
		std::cout << "  unsigned short-> " << toCharString(us) << std::endl;
		std::cout << "  int           -> " << toCharString(i) << std::endl;
		std::cout << "  unsigned int  -> " << toCharString(ui) << std::endl;
		std::cout << "  long          -> " << toCharString(l) << std::endl;
		std::cout << "  unsigned long -> " << toCharString(ul) << std::endl;
		std::cout << "  long long     -> " << toCharString(ll) << std::endl;
		std::cout << "  unsigned llong-> " << toCharString(ull) << std::endl;
		std::cout << "  float         -> " << toCharString(f) << std::endl;
		std::cout << "  double        -> " << toCharString(d) << std::endl;
		std::cout << "  float(2)      -> " << toCharString(f,2) << std::endl;
		std::cout << "  double(2)     -> " << toCharString(d,2) << std::endl;
	    std::cout << "  float (w/ buf)-> " << floatToCharString(f) << std::endl;
    }
    SUCCEED();
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc,argv);
	return RUN_ALL_TESTS();
}

