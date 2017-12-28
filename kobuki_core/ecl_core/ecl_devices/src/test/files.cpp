/**
 * @file /src/test/files.cpp
 *
 * @brief Unit Test for the file devices.
 *
 * @date August 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <gtest/gtest.h>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/containers/array.hpp>
#include "../../include/ecl/devices/detail/character_buffer.hpp"
#include "../../include/ecl/devices/ofile.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::Append;
using ecl::devices::CharStringBuffer;
using ecl::New;
using ecl::OFile;
using ecl::StandardException;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(FilesTests,construct) {

	bool result = true;
	try {
		OFile o_file1("odude.txt",New);
	    OFile o_file2;
	    o_file2.open("odude2.txt",New);
	} catch ( const StandardException& e ) {
		result = false;
	}
	EXPECT_TRUE(result);
}

TEST(FilesTests,write) {
	OFile o_file1("odude.txt",New);
    OFile o_file2;
    o_file2.open("odude2.txt",New);
	long n;
	n = o_file1.write("Heya Dude\n",10);
	o_file1.flush();
	EXPECT_EQ(10,n);
	n = o_file2.write("Heya Dude\n",10);
	o_file2.flush();
	EXPECT_EQ(10,n);
	std::string heya_dude("Heya Dude From Array\n");
	ecl::Array<char,256> buffer;
	std::copy(heya_dude.begin(), heya_dude.end(), buffer.begin());
	n = o_file2.write(buffer.stencil(0,heya_dude.size()));
	o_file2.flush();
	EXPECT_EQ(21,n);
// This is an interesting test of the buffer which sometimes gets autoflushed
//  for ( unsigned int i = 0; i < CharStringBuffer::buffer_size/10 + 1; ++i ) {
//	    o_file1.write("Heya Dude\n",10);
//	    o_file1.flush();
//	    o_file2.write("Heya Dude\n",10);
//	    o_file2.flush();
//  }
}

TEST(FilesTests,append) {
    OFile o_file("odude.txt",Append);
	long n;
	n = o_file.write("Appending Dude\n",15);
	EXPECT_EQ(15,n);
    EXPECT_TRUE( o_file.flush());
}

TEST(FilesTest,isOpen) {
    OFile o_file1("odude2.txt",Append);
    EXPECT_TRUE( o_file1.open());
    OFile o_file2;
    EXPECT_FALSE( o_file2.open());
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}

