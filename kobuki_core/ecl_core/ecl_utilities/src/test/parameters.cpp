/**
 * @file /src/test/parameters.cpp
 *
 * @brief Unit Test for the @ref ecl::Parameter "Parameter" class.
 *
 * @date April 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include "../../include/ecl/utilities/parameter.hpp"

/**
 * @cond DO_NOT_DOXYGEN
 */

/*****************************************************************************
 * Classes
 ****************************************************************************/

namespace ecl {
namespace utilities {
namespace tests {


class ParameterTest {
public:
	ParameterTest() : value(3) {};
    Parameter<int> value;
};

}}}

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::Parameter;
using ecl::utilities::tests::ParameterTest;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(Parameter,get) {

	ParameterTest test;
    int i1 = test.value();
    // int &i2 = test.value(); // This one is not permitted the class loses control of the variable.
    const int i3 = test.value();
    const int &i4 = test.value();
    EXPECT_EQ(3,i1);
    EXPECT_EQ(3,i3);
    EXPECT_EQ(3,i4);
}

TEST(Parameter,set) {

	ParameterTest test;
    int i1 = test.value();
	int i_tmp = 3;
	int &i2 = i_tmp;
	const int i3 = test.value();
	const int &i4 = test.value();
	test.value(3);      // temporary
	EXPECT_EQ(3,test.value());
	test.value(i1);     // int
	EXPECT_EQ(3,test.value());
	test.value(i2);     // int&
	EXPECT_EQ(3,test.value());
	test.value(i3);     // const int
	EXPECT_EQ(3,test.value());
	test.value(i4);     // const int&
	EXPECT_EQ(3,test.value());
}

TEST(Parameter,assign) {

	ParameterTest test;
    int i1 = test.value();
	int i_tmp = 3;
	int &i2 = i_tmp;
	const int i3 = test.value();
	const int &i4 = test.value();

	test.value = 3;      // temporary
	EXPECT_EQ(3,test.value());
    test.value = i1;     // int
	EXPECT_EQ(3,test.value());
    test.value = i2;     // int&
	EXPECT_EQ(3,test.value());
    test.value = i3;     // const int
	EXPECT_EQ(3,test.value());
    test.value = i4;     // const int&
	EXPECT_EQ(3,test.value());
}

TEST(Parameter,eval) {

	ParameterTest test;
	int i1;
	i1 = test.value;
	EXPECT_EQ(3,i1);
//    int &i5 = test.value; // This one is not permitted the class loses control of the variable.
    const int &i5 = test.value;
	EXPECT_EQ(3,i5);
    const int i6 = test.value;
	EXPECT_EQ(3,i6);
    i1 = i5;  // remove the warning
	EXPECT_EQ(3,i1);
    i1 = i6;  // remove the warning
	EXPECT_EQ(3,i1);
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc,argv);
	return RUN_ALL_TESTS();
}
/**
 * @endcond
 */
