/**
 * @file /src/test/string_streams.cpp
 *
 * @brief Unit Test for the string stream.
 *
 * @date December 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <string>
#include <gtest/gtest.h>
#include "../../include/ecl/streams/string_stream.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using std::string;
using ecl::StringStream;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(StringStreams,floatTypes) {
    StringStream sstream;
    float f = 33.33;
    double d = -1.0;
    sstream << f;
    sstream >> d;
    EXPECT_FALSE(sstream.fail());
    // Give it some room for rounding off
    EXPECT_GT(33.34,d);
    EXPECT_LT(33.32,d);
}

TEST(StringStreams,clearStream) {
    StringStream sstream;
    sstream << "dude";
    sstream.clear();
    sstream << "aha";
    EXPECT_FALSE(sstream.fail());
    EXPECT_EQ(string("aha"),sstream.str());
}

TEST(StringStreams,integralTypes) {
    StringStream sstream;
    int i = 3;
    int j = -1;
    sstream << i;
    sstream >> j;
    EXPECT_FALSE(sstream.fail());
    // Give it some room for rounding off
    EXPECT_EQ(3,j);
}

TEST(StringStreams,hexTypes) {
    StringStream sstream;
    int j = -1;
    sstream << "0xA0";
    sstream >> j;
    EXPECT_FALSE(sstream.fail());
    // Give it some room for rounding off
    EXPECT_EQ(160,j);
}
TEST(StringStreams,multipleOps) {
	StringStream sstream;
    string s;
    sstream << "Hello";
    sstream >> s;
    EXPECT_FALSE(sstream.fail());
    EXPECT_EQ(string("Hello"),s);
    sstream << "Dude";
    sstream >> s;
    EXPECT_FALSE(sstream.fail());
    EXPECT_EQ(string("Dude"),s);
}

TEST(StringStreams,readEmptyFail) {
	StringStream sstream;
	int j;
    sstream >> j;
    EXPECT_TRUE(sstream.fail());
}

TEST(StringStreams,remaining) {
	StringStream sstream;
    sstream << "Dude";
    EXPECT_EQ(4,sstream.device().remaining());
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc,argv);
	return RUN_ALL_TESTS();
}


