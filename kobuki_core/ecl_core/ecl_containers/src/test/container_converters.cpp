/**
 * @file /ecl_containers/src/test/converters.cpp
 *
 * @brief Unit test the stencil char byte array -> integral type converter.
 *
 * @date August 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <gtest/gtest.h>
#include "../../include/ecl/containers/array.hpp"
#include "../../include/ecl/containers/converters.hpp"
#include "../../include/ecl/containers/stencil.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::Array;
using ecl::Stencil;

/*****************************************************************************
** Globals
*****************************************************************************/

bool debug_output = true;

/*****************************************************************************
** Functions
*****************************************************************************/

template <typename Container>
void print(const Container &byte_array) {
	if( debug_output ) {
		std::cout << byte_array << std::endl;
	}
}

/*****************************************************************************
** Typedefs
*****************************************************************************/

typedef Array<char,3> ByteArray;
typedef Array<char> DynamicByteArray;
typedef Stencil< Array<char,3> > ByteStencil;
typedef Stencil< Array<char> > DynamicByteStencil;
typedef Array<signed char,3> SignedByteArray;
typedef Array<signed char> DynamicSignedByteArray;
typedef Stencil< Array<signed char,3> > SignedByteStencil;
typedef Stencil< Array<signed char> > DynamicSignedByteStencil;
typedef Array<unsigned char,3> UnsignedByteArray;
typedef Array<unsigned char> DynamicUnsignedByteArray;
typedef Stencil< Array<unsigned char,3> > UnsignedByteStencil;
typedef Stencil< Array<unsigned char> > DynamicUnsignedByteStencil;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(ConverterTests,fromArray) {
	ByteArray byte_array;
	byte_array << 0x01, 0x02, 0x03;
	ecl::Converter<int,ByteArray> toInt;
	if( debug_output ) { std::cout << "Conversion: " << toInt(byte_array) << std::endl; }
	EXPECT_EQ(197121,toInt(byte_array));
	UnsignedByteArray unsigned_byte_array;
	unsigned_byte_array << 0x01, 0x02, 0x03;
	ecl::Converter<int,UnsignedByteArray> utoInt;
	if( debug_output ) { std::cout << "Conversion: " << utoInt(unsigned_byte_array) << std::endl; }
	EXPECT_EQ(197121,utoInt(unsigned_byte_array));
	SignedByteArray signed_byte_array;
	signed_byte_array << 0x01, 0x02, 0x03;
	ecl::Converter<int,SignedByteArray> stoInt;
	if( debug_output ) { std::cout << "Conversion: " << stoInt(signed_byte_array) << std::endl; }
	EXPECT_EQ(197121,stoInt(signed_byte_array));
}

TEST(ConverterTests,toArray) {
	// Only works for dynamic containers!
	ecl::int32 i = 197121;
	ecl::Converter<DynamicByteArray,ecl::int32> toByteArray;
	ecl::Converter<DynamicSignedByteArray,ecl::int32> toSignedByteArray;
	ecl::Converter<DynamicUnsignedByteArray,ecl::int32> toUnsignedByteArray;
	DynamicByteArray byte_array(4);
	DynamicSignedByteArray signed_byte_array(4);
	DynamicUnsignedByteArray unsigned_byte_array(4);
	toByteArray(byte_array, i);
	toSignedByteArray(signed_byte_array,i);
	toUnsignedByteArray(unsigned_byte_array, i);
	print(byte_array);
	print(signed_byte_array);
	print(unsigned_byte_array);
    EXPECT_EQ(1,byte_array[0]);
    EXPECT_EQ(2,byte_array[1]);
    EXPECT_EQ(3,byte_array[2]);
    EXPECT_EQ(1,signed_byte_array[0]);
    EXPECT_EQ(2,signed_byte_array[1]);
    EXPECT_EQ(3,signed_byte_array[2]);
    EXPECT_EQ(1,unsigned_byte_array[0]);
    EXPECT_EQ(2,unsigned_byte_array[1]);
    EXPECT_EQ(3,unsigned_byte_array[2]);

}
TEST(ConverterTests,fromStencil) {
	ByteArray byte_array;
	SignedByteArray signed_byte_array;
	UnsignedByteArray unsigned_byte_array;
	byte_array << 0x01, 0x02, 0x03;
	signed_byte_array << 0x01, 0x02, 0x03;
	unsigned_byte_array << 0x01, 0x02, 0x03;
	ecl::Converter<int,ByteStencil> toInt;
	ecl::Converter<int,SignedByteStencil> stoInt;
	ecl::Converter<int,UnsignedByteStencil> utoInt;
	int i = toInt(byte_array.stencil(0,2));
	int si = stoInt(signed_byte_array.stencil(0,2));
	int ui = utoInt(unsigned_byte_array.stencil(0,2));
	if( debug_output ) { std::cout << "Conversion: " << std::dec << i << std::endl; }
	if( debug_output ) { std::cout << "Conversion: " << std::dec << si << std::endl; }
	if( debug_output ) { std::cout << "Conversion: " << std::dec << ui << std::endl; }
	EXPECT_EQ(513,i); // first two bytes only
	EXPECT_EQ(513,si); // first two bytes only
	EXPECT_EQ(513,ui); // first two bytes only
}
TEST(ConverterTests,toStencil) {
	Array<char,20> byte_array = Array<char,20>::Constant(0x00);
	Stencil< Array<char, 20> > stencil = byte_array.stencil(2,4);
	ecl::Converter< Stencil< Array<char, 20> >, ecl::int32 > fromInt;
	stencil = fromInt(stencil, 197121);
	print(byte_array);
	print(stencil);
    EXPECT_EQ(1,stencil[0]);
    EXPECT_EQ(2,stencil[1]);
    EXPECT_EQ(3,stencil[2]);
}
/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}

