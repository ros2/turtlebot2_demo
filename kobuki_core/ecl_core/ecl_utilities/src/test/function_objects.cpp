/**
 * @file /src/test/function_objects.cpp
 *
 * @brief Unit Test for the function objects.
 *
 * @date June 29, 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include "../../include/ecl/utilities/function_objects.hpp"

/**
 * @cond DO_NOT_DOXYGEN
 */

/*****************************************************************************
** Classes
*****************************************************************************/

namespace ecl {
namespace utilities {
namespace tests {

class FunctionObject {
public:
	FunctionObject() : result(-1) {}
	typedef void result_type;
	void operator()() {
		result = 0;
//		std::cout << "Void function object." << std::endl;
	}
	int result;
};

class MemberFunctions {
public:
	void e() {
		result = 0;
//	    std::cout << "Member function with no arguments." << std::endl;;
	}
	void f(const int &i) {
		result = 1;
//	    std::cout << "Member function with argument: " << i << std::endl;;
	}
	void g() {
		result = 2;
//	    std::cout << "Member function" << std::endl;;
	}
	void h(const int& i) {
		result = 3;
//	    std::cout << "Member function with ref argument: " << i << std::endl;;
	}
	int result;
};

/*****************************************************************************
** Functions
*****************************************************************************/

static int free_result = -1;

void rnf1() {
	free_result = 0;
//    std::cout << "Free nullary function" << std::endl;;
}

int rnf2() {
	free_result = 1;
//    std::cout << "Free nullary function returning: 3" << std::endl;;
    return 3;
}

void ruf1(const int &i) {
	free_result = 2;
//    std::cout << "Free unary function with argument: " << i << std::endl;;
}

int ruf2(const int &i) {
	free_result = 3;
//    std::cout << "Free unary function with argument: " << i << std::endl;;
    return i;
}

}}}

/*****************************************************************************
** Using
*****************************************************************************/

using namespace ecl;
using namespace ecl::utilities::tests;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(FunctionObjects,freeFunctions) {
    NullaryFreeFunction<void> rnfo1(rnf1);
    rnfo1();
    EXPECT_EQ(0,free_result);

    NullaryFreeFunction<int> rnfo2(rnf2);
    NullaryFreeFunction<int>::result_type i1 = rnfo2();
    EXPECT_EQ(3,i1);

    UnaryFreeFunction<const int&,void> rufo1(ruf1);
    rufo1(2);
    EXPECT_EQ(2,free_result);

    UnaryFreeFunction<const int&,int> rufo2(ruf2);
    UnaryFreeFunction<int,int>::result_type i2 = rufo2(4);
    EXPECT_EQ(4,i2);
}

TEST(FunctionObjects,boundFreeFunctions) {

    BoundUnaryFreeFunction<const int&,void> brufo1(ruf1,5);
    brufo1();
    EXPECT_EQ(2,free_result);
}

TEST(FunctionObjects,memberFunctions) {

    MemberFunctions a;
    NullaryMemberFunction<MemberFunctions,void> mufo1(&MemberFunctions::e);
    mufo1(a);
    EXPECT_EQ(0,a.result);

    UnaryMemberFunction<MemberFunctions,const int&,void> mbfo1(&MemberFunctions::f);
    mbfo1(a,2);
    EXPECT_EQ(1,a.result);
}

TEST(FunctionObjects,boundMemberFunctions) {

    MemberFunctions a;
    BoundNullaryMemberFunction<MemberFunctions,void> bmufo1(&MemberFunctions::e, a);
    bmufo1();
    EXPECT_EQ(0,a.result);

    BoundUnaryMemberFunction<MemberFunctions,const int&,void> bmbfo1(&MemberFunctions::f, a, 3);
    bmbfo1();
    EXPECT_EQ(1,a.result);
}

TEST(FunctionObjects,wrappers) {

    FunctionObject function_object;
    NullaryFunctionCopy<FunctionObject> nfcopy(function_object);
    nfcopy();
    // Can't really validate this baby.

//	std::cout << "Nullary Function References." << std::endl;
	NullaryFunctionReference<FunctionObject> nfref(ref(function_object));
    nfref();
    // Can't really validate this baby.
}

TEST(FunctionObjects,generators) {

    MemberFunctions a;
	generateFunctionObject(rnf1)();
	EXPECT_EQ(0,free_result);
    generateFunctionObject(ruf1)(3);
	EXPECT_EQ(2,free_result);
    generateFunctionObject(ruf1,3)();
	EXPECT_EQ(2,free_result);
    generateFunctionObject(&MemberFunctions::e)(a);
    EXPECT_EQ(0,a.result);
    generateFunctionObject(&MemberFunctions::e, a)();
    EXPECT_EQ(0,a.result);
    generateFunctionObject(&MemberFunctions::f)(a,1);
    EXPECT_EQ(1,a.result);
    generateFunctionObject(&MemberFunctions::h)(a,1);
    EXPECT_EQ(3,a.result);
    generateFunctionObject(&MemberFunctions::h,a,1)();  // Calls the non const version
    EXPECT_EQ(3,a.result);
    int j = 3;
    generateFunctionObject(&MemberFunctions::h,a,j)();  // Calls the const version
    EXPECT_EQ(3,a.result);
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
