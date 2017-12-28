/**
 * @file /src/test/file_streams.cpp
 *
 * @brief Unit Test for the file streams.
 *
 * @date September, 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <gtest/gtest.h>
#include <ecl/devices/ofile.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include "../../include/ecl/streams/file_streams.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using std::string;
using ecl::OFile;
using ecl::Append;
using ecl::New;
using ecl::StandardException;
using ecl::TextStream;
using ecl::OFileStream;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(OFileStreamTests,construct) {
    OFileStream ostream("dude_stream.txt",New);
    EXPECT_TRUE(ostream.device().open());
}

TEST(OFileStreamTests,writeChar) {
    OFileStream ostream("dude_stream.txt",Append);
    bool result = true;
    try {
    	ostream << 'c' << '\n';
        ostream.flush();
    } catch ( const StandardException &e ) {
    	result = false;
    }
    EXPECT_TRUE(result);
}

TEST(OFileStreamTests,writeCharString) {
    OFileStream ostream("dude_stream.txt",Append);
    bool result = true;
    try {
        ostream << "Dude\n";
        ostream.flush();
    } catch ( const StandardException &e ) {
    	result = false;
    }
    EXPECT_TRUE(result);
}

TEST(OFileStreamTests,writeString) {
    OFileStream ostream("dude_stream.txt",Append);
    bool result = true;
    try {
        string dude("dude_string\n");
        ostream << dude;
        ostream.flush();
    } catch ( const StandardException &e ) {
    	result = false;
    }
    EXPECT_TRUE(result);
}

TEST(OFileStreamTests,writeIntegralTypes) {
    OFileStream ostream("dude_stream.txt",Append);
    bool result = true;
    try {
        short si = 1;
        ostream << si << '\n';
        int i = 2;
        ostream << i << '\n';
        long l = 3;
        ostream << l << '\n';
        long long ll = 4;
        ostream << ll << '\n';
        unsigned short us = 5;
        ostream << us << '\n';
        unsigned int ui = 6;
        ostream << ui << '\n';
        unsigned long ul = 77;
        ostream << ul << '\n';
        unsigned long long ull = 8888;
        ostream << ull << '\n';
        ostream << 77 << '\n';
        ostream.flush();
    } catch ( const StandardException &e ) {
    	result = false;
    }
    EXPECT_TRUE(result);
}

TEST(OFileStreamTests,writeBool) {
    OFileStream ostream("dude_stream.txt",Append);
    bool result = true;
    try {
        bool test = true;
        ostream << test << '\n';
        ostream << false << '\n';
        ostream.flush();
    } catch ( const StandardException &e ) {
    	result = false;
    }
    EXPECT_TRUE(result);
}

TEST(OFileStreamTests,writeFloatTypes) {
    OFileStream ostream("dude_stream.txt",Append);
    bool result = true;
    try {
        float f = 32.1;
        double d = -33.3;
        ostream << f << '\n';
        ostream << d << '\n';
        ostream.flush();
    } catch ( const StandardException &e ) {
    	result = false;
    }
    EXPECT_TRUE(result);
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc,argv);
	return RUN_ALL_TESTS();
}
