/**
 * @file /src/test/stencil.cpp
 *
 * @brief Unit Test for the @ref ecl::Stencil "Stencil" class.
 *
 * @date March, 2010
 **/
/*****************************************************************************
** Macros
*****************************************************************************/

// Make sure we enter debug mode.
#ifdef NDEBUG
  #undef NDEBUG
#endif
#ifdef ECL_NDEBUG
  #undef ECL_NDEBUG
#endif

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <gtest/gtest.h>
//#include </ecl/config/ecl.hpp>
#include "../../include/ecl/containers/array.hpp"
#include "../../include/ecl/containers/stencil.hpp"
#include <ecl/concepts/containers.hpp>

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::StandardException;
using ecl::Array;
using ecl::Stencil;
using ecl::UnsignedByteContainerConcept;

/*****************************************************************************
** Globals
*****************************************************************************/

typedef Stencil< Array<int,5> > FixedStencil;
typedef Stencil< Array<int> > DynamicStencil;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(StencilTests,construction) {

	Array<int,5> array;
	Array<int> darray(5);
	array << 1,2,3,4,5;
	darray << 6,7,8,9,10;

	FixedStencil stencil = array.stencil(1,3); // 2,3.4
	DynamicStencil dstencil(darray,darray.begin()+1,darray.begin()+4); // 7,8,9
	EXPECT_EQ(2,stencil[0]);
	EXPECT_EQ(3,stencil[1]);
	EXPECT_EQ(4,stencil[2]);
	EXPECT_EQ(7,dstencil[0]);
	EXPECT_EQ(8,dstencil[1]);
	EXPECT_EQ(9,dstencil[2]);
}

TEST(StencilTests,badConstruction) {

	Array<int> darray(5);
	darray << 6,7,8,9,10;

	bool result = false;
    try {
    	DynamicStencil bad_stencil(darray,darray.begin()+1,darray.begin()+9);
    } catch ( StandardException &e ) {
    	result = true;
    }
    EXPECT_TRUE(result);
}

TEST(StencilTests,badIndexing) {

	Array<int> darray(5);
	darray << 6,7,8,9,10;

	DynamicStencil dstencil(darray,darray.begin()+1,darray.begin()+4); // 7,8,9
	darray.resize(3);
	darray << 6,7,8;
	bool result = false;
    try {
    	dstencil[2]; // Even something that looks good will generally fail as the memory has moved.
    	dstencil[4];
    } catch ( StandardException &e ) {
    	result = true;
    }
    EXPECT_TRUE(result);
}
TEST(StencilTests,modifying) {
	Array<int,5> array;
	Array<int> darray(5);
	array << 1,2,3,4,5;
	darray << 6,7,8,9,10;
	FixedStencil stencil = array.stencil(1,3); // 2,3.4
	stencil.resettle(0,2); // 1,2
	EXPECT_EQ(1,stencil[0]);
	EXPECT_EQ(2,stencil[1]);
	stencil = array.stencil(1,4); // 2,3,4,5
	EXPECT_EQ(2,stencil[0]);
	EXPECT_EQ(3,stencil[1]);
	EXPECT_EQ(4,stencil[2]);
	EXPECT_EQ(5,stencil[3]);
}

TEST(StencilTests,concepts) {

	typedef Array<unsigned char,6> ByteArray;
    typedef Stencil<ByteArray> ByteStencil;

    // ecl_compile_time_concept_check(ecl::concepts::ByteContainer<FixedStencil>); // this one fails
    ecl_compile_time_concept_check(UnsignedByteContainerConcept<ByteArray>);
    ecl_compile_time_concept_check(UnsignedByteContainerConcept<ByteStencil>);
    SUCCEED();
}

TEST(StencilTests,byteStrings) {

	std::string str("dude");
    Stencil<std::string> stencil( str, str.begin(), str.end() );
	EXPECT_EQ(4,stencil.size());
	EXPECT_EQ('d',stencil[0]);
	EXPECT_EQ('u',stencil[1]);
	EXPECT_EQ('d',stencil[2]);
	EXPECT_EQ('e',stencil[3]);
//    for ( unsigned int i = 0; i < stencil.size(); ++i ) {
//    	std::cout << i << ": " << stencil[i] << std::endl;
//    }

    SUCCEED();
}

TEST(StencilTests,unsignedCharArrays) {

    //unsigned char buffer[4] = { 0xff, 0x00, 0x01, 0x02 };
    unsigned char buffer[4];
    Stencil<unsigned char*> stencil( buffer, 4, buffer, buffer + 4);
    stencil << 0xff, 0x00, 0x01, 0x02;
    EXPECT_EQ(4,stencil.size());
    EXPECT_EQ(255,stencil[0]);
    EXPECT_EQ(0,stencil[1]);
    EXPECT_EQ(1,stencil[2]);
    EXPECT_EQ(2,stencil[3]);
//    std::cout << std::hex;
//    for ( unsigned int i = 0; i < stencil.size(); ++i ) {
//      std::cout << i << ": " << static_cast<int>(stencil[i]) << std::endl;
//    }
//    std::cout << std::dec;

    SUCCEED();
}


/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}


