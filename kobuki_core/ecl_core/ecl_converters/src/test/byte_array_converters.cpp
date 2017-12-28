/**
 * @file /src/test/byte_array_converters.cpp
 *
 * @brief Unit test for the byte array converters.
 *
 * @date August 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <vector>
#include <gtest/gtest.h>
#include <ecl/config/portable_types.hpp>
#include "../../include/ecl/converters/to_byte_array.hpp"
#include "../../include/ecl/converters/from_byte_array.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::Converter;

/*****************************************************************************
** Typedefs
*****************************************************************************/

typedef std::vector<unsigned char> ByteArray;

/*****************************************************************************
** Globals
*****************************************************************************/

bool debug_output = false;

void print(const ByteArray &byte_array) {
	if( debug_output ) {
		for ( unsigned int i = 0; i < byte_array.size(); ++i ) {
			std::cout << "0x" << std::hex << static_cast<unsigned int>(byte_array[i]) << " ";
		}
		std::cout << std::endl;
	}
}

void print(const std::vector<char> &byte_array) {
	if( debug_output ) {
		for ( unsigned int i = 0; i < byte_array.size(); ++i ) {
			std::cout << "0x" << std::hex << static_cast<unsigned int>(byte_array[i]) << " ";
		}
		std::cout << std::endl;
	}
}
/*****************************************************************************
** Tests
*****************************************************************************/

TEST(Converter,fromByteArray) {
	ByteArray byte_array;
	byte_array.push_back(0x01);
	byte_array.push_back(0x02);
	byte_array.push_back(0x03);
	Converter<int,ByteArray> converter;
//	std::cout << "Byte Array Conversion : " << converter(byte_array) << std::endl;
    EXPECT_EQ(197121,converter(byte_array));
}

TEST(Converter,toByteArray) {
	ecl::Converter< ByteArray, int > toByteArray;
	ByteArray byte_array(4,0x00);
	toByteArray(byte_array, 363);
	print(byte_array);
    EXPECT_EQ(107,byte_array[0]);
    toByteArray(byte_array,-2);
	print(byte_array);
    EXPECT_EQ(254,byte_array[0]);
    ecl::Converter< int, ByteArray > fromByteArray;
    int i = fromByteArray(byte_array);
    if ( debug_output ) {
    	std::cout << std::dec << "i: " << i << std::endl;
    }
    EXPECT_EQ(-2,i);
    toByteArray(byte_array, 258);
	print(byte_array);
    i = fromByteArray(byte_array);
    if ( debug_output ) {
    	std::cout << std::dec << "i: " << i << std::endl;
    }
}
TEST(Converter,toByteArrayFailedSize) {
  #if defined(NDEBUG) || defined(ECL_NDEBUG)
    std::cout << "Skipping toByteArrayFailedSize test, it only throws in debug mode" << std::endl;
    SUCCEED();
  #else
    ecl::Converter< ByteArray, ecl::int32 > toByteArray;
    ecl::int32 i = 363;
    ByteArray byte_array;
    try {
      byte_array = toByteArray(byte_array, i);
    } catch ( ecl::StandardException &e ) {
      if ( debug_output ) { std::cout << "Caught an exception from improper size of reserved memory." << std::endl; }
    }
    ecl::Error result = toByteArray.error();
    if ( result.flag() == ecl::ConversionError ) {
      SUCCEED();
    } else {
      FAIL();
    }
  #endif
}

TEST(Converter,charToByteArray) {
    Converter< std::vector<char> > toCharByteArray;
    std::vector<char> bytes = toCharByteArray("0x32 0x54");
    if(debug_output) {
    	print(bytes);
    }
    EXPECT_EQ(50,bytes[0]);
    EXPECT_EQ(84,bytes[1]);
    std::vector<char> bytes_258(4,0x00);
    toCharByteArray(bytes_258, ecl::int32(258));
    print(bytes_258);
    ecl::Converter< ecl::int32, std::vector<char> > fromCharByteArray;
    ecl::int32 i = fromCharByteArray(bytes_258);
    if ( debug_output ) {
    	std::cout << std::dec << "i: " << i << std::endl;
    }
}

TEST(Converter,byteArrayStringConverters) {
	std::string str("dude");
	ecl::converters::FromByteArray<ecl::int32,std::string> from_byte_array;
	int i = from_byte_array(str);
    if ( debug_output ) {
    	std::cout << "i: " << i << std::endl;
    }
    // is this correct? I haven't actually verified this.
	EXPECT_EQ(1701082468,i);
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}



