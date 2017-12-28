/**
 * @file /ecl_type_traits/src/test/fundamental_types.cpp
 *
 * @brief Unit tests for fundamental_type traits.
 *
 * @date July 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include <limits>
#include <ecl/config/ecl.hpp>
#include "../../include/ecl/type_traits/fundamental_types.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::is_integral;
using ecl::is_float;
using ecl::is_byte;
using ecl::is_signed_byte;
using ecl::is_unsigned_byte;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(TypeTraitTests,is_integral) {
    bool result;
    result = is_integral<short>::value;
    EXPECT_EQ(true, result);
    result = is_integral<unsigned short>::value;
    EXPECT_EQ(true, result);
    result = is_integral<int>::value;
    EXPECT_EQ(true, result);
    result = is_integral<unsigned int>::value;
    EXPECT_EQ(true, result);
    result = is_integral<long>::value;
    EXPECT_EQ(true, result);
    result = is_integral<unsigned long>::value;
    EXPECT_EQ(true, result);
    result = is_integral<long long>::value;
    EXPECT_EQ(true, result);
    result = is_integral<unsigned long long>::value;
    EXPECT_EQ(true, result);
    result = is_integral<float>::value;
    EXPECT_EQ(false, result);
    result = is_integral<double>::value;
    EXPECT_EQ(false, result);
}

TEST(TypeTraitTests,is_float) {
    bool result;
    result = is_float<short>::value;
    EXPECT_EQ(false, result);
    result = is_float<unsigned short>::value;
    EXPECT_EQ(false, result);
    result = is_float<int>::value;
    EXPECT_EQ(false, result);
    result = is_float<unsigned int>::value;
    EXPECT_EQ(false, result);
    result = is_float<long>::value;
    EXPECT_EQ(false, result);
    result = is_float<unsigned long>::value;
    EXPECT_EQ(false, result);
    result = is_float<long long>::value;
    EXPECT_EQ(false, result);
    result = is_float<unsigned long long>::value;
    EXPECT_EQ(false, result);
    result = is_float<float>::value;
    EXPECT_EQ(true, result);
    result = is_float<double>::value;
    EXPECT_EQ(true, result);
}

TEST(TypeTraitTests,is_byte) {
    bool result;

    result = is_byte<char>::value;
    EXPECT_EQ(true, result);
    result = is_byte<signed char>::value;
    EXPECT_EQ(true, result);
    result = is_byte<unsigned char>::value;
    EXPECT_EQ(true, result);

    result = is_signed_byte<char>::value;
    if ( std::numeric_limits<char>::is_signed ) {
        EXPECT_EQ(true, result);
    } else {
        EXPECT_EQ(false, result);
    }
    result = is_signed_byte<signed char>::value;
    EXPECT_EQ(true, result);
    result = is_signed_byte<unsigned char>::value;
    EXPECT_EQ(false, result);

    result = is_unsigned_byte<char>::value;
    if ( std::numeric_limits<char>::is_signed ) {
        EXPECT_EQ(false, result);
    } else {
        EXPECT_EQ(true, result);
    }
    result = is_unsigned_byte<signed char>::value;
    EXPECT_EQ(false, result);
    result = is_unsigned_byte<unsigned char>::value;
    EXPECT_EQ(true, result);
}
/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
