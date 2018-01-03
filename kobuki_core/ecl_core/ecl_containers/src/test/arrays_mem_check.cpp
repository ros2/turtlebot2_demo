/**
 * @file /src/test/arrays_mem_check.cpp
 *
 * @brief Unit Test for ecl::Array containers.
 *
 * @date September 2009
 **/
/*****************************************************************************
** Macros
*****************************************************************************/

#define ECL_MEM_CHECK_ARRAYS // Make sure we use mem checking arrays, not regular

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <gtest/gtest.h>
#include "../../include/ecl/containers/array.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::Array;
using ecl::blueprints::ArrayFactory;
using ecl::blueprints::ConstantArray;
using ecl::StandardException;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(ArrayMemCheckTests,constructors) {
    Array<int,4> array1 = Array<int,4>::Constant(3);
    EXPECT_EQ(3,array1[0]);
    EXPECT_EQ(3,array1[1]);
    EXPECT_EQ(3,array1[2]);
    EXPECT_EQ(3,array1[3]);
    Array<int,4> array2 = ArrayFactory<int,4>::Constant(4);
    EXPECT_EQ(4,array2[0]);
    EXPECT_EQ(4,array2[1]);
    EXPECT_EQ(4,array2[2]);
    EXPECT_EQ(4,array2[3]);
    Array<int,4> array3 = ConstantArray<int,4>(3);
    EXPECT_EQ(3,array3[0]);
    EXPECT_EQ(3,array3[1]);
    EXPECT_EQ(3,array3[2]);
    EXPECT_EQ(3,array3[3]);
//    std::cout << "Constant array [fixed]  : " << array1 << std::endl;
    Array<int> darray1 = Array<int>::Constant(4,3);
    EXPECT_EQ(3,darray1[0]);
    EXPECT_EQ(3,darray1[1]);
    EXPECT_EQ(3,darray1[2]);
    EXPECT_EQ(3,darray1[3]);
//    std::cout << "Constant array [dynamic]  : " << darray1 << std::endl;
}

TEST(ArrayMemCheckTests,copyConstructors) {

    Array<int,4> array1 = Array<int,4>::Constant(3);
    Array<int> darray1 = Array<int>::Constant(4,3);

    Array<int,4> array2(array1);
	Array<int> darray2(darray1);

    EXPECT_EQ(3,array2[0]);
    EXPECT_EQ(3,array2[1]);
    EXPECT_EQ(3,array2[2]);
    EXPECT_EQ(3,array2[3]);

    EXPECT_EQ(3,darray2[0]);
    EXPECT_EQ(3,darray2[1]);
    EXPECT_EQ(3,darray2[2]);
    EXPECT_EQ(3,darray2[3]);
}

TEST(ArrayMemCheckTests,blueprintAssignment) {

	Array<int,4> array;
	array = Array<int,4>::Constant(3);

    EXPECT_EQ(3,array[0]);
    EXPECT_EQ(3,array[1]);
    EXPECT_EQ(3,array[2]);
    EXPECT_EQ(3,array[3]);
}
TEST(ArrayMemCheckTests,commaInitialisation) {

	Array<int,4> array;
    Array<int> darray(4);
	array << 1,2,3,4;
	darray << 1,2,3,4;

    EXPECT_EQ(1,array[0]);
    EXPECT_EQ(2,array[1]);
    EXPECT_EQ(3,array[2]);
    EXPECT_EQ(4,array[3]);

    EXPECT_EQ(1,darray[0]);
    EXPECT_EQ(2,darray[1]);
    EXPECT_EQ(3,darray[2]);
    EXPECT_EQ(4,darray[3]);
}
TEST(ArrayMemCheckTests,Accessors) {

	int j;
	Array<int,4> array;
    Array<int> darray(4);
	array << 1,2,3,4;
	darray << 1,2,3,4;

	j = array[2];
	EXPECT_EQ(3,j);
	j = array.at(1);
	EXPECT_EQ(2,j);
	j = darray[2];
	EXPECT_EQ(3,j);
	j = darray.at(1);
	EXPECT_EQ(2,j);
}

TEST(ArrayMemCheckTests,rangeChecking) {

	Array<int,4> array = Array<int,4>::Constant(3);
    Array<int> darray = Array<int>::Constant(4,3);
	bool result = false;
	try {
        array.at(10);
    } catch ( StandardException &e ) {
    	result = true;
//        std::cout << "Fixed:\n" << e.what() << std::endl;
    }
    EXPECT_TRUE(result);
    result = false;
    try {
        darray.at(10);
    } catch ( StandardException &e ) {
    	result = true;
//        std::cout << "Dynamic:\n" << e.what() << std::endl;
    }
    EXPECT_TRUE(result);
}

TEST(ArrayMemCheckTests,iterators) {

    Array<int,4> array = Array<int,4>::Constant(3);
	Array<int,4>::iterator iterator;
	for ( iterator = array.begin(); iterator != array.end(); ++iterator ) {
		EXPECT_EQ(3,*iterator);
	}
}

TEST(ArrayMemCheckTests,resize) {
    Array<int> darray = Array<int>::Constant(4,3);
    darray.resize(10);
    darray << 0,1,2,3,4,5,6,7,8,9;
    int i = 0;
    Array<int>::iterator iterator;
	for ( iterator = darray.begin(); iterator != darray.end(); ++iterator ) {
		EXPECT_EQ(i,*iterator);
		++i;
	}
	darray.clear();
}

TEST(ArrayMemCheckTests,bufferRuns) {
    int *istart = NULL;
    Array<int,4> array_ok; array_ok << 1, 2, 3, 4;
    Array<int,4> array_under; array_under << 1, 2, 3, 4;
    Array<int,4> array_over; array_over << 1, 2, 3, 4;

//    std::cout << "Fixed Arrays" << std::endl;
//    std::cout << "Array_under/over/ok: " << array_ok << std::endl;
//

//    std::cout << "Underrunning array_under." << std::endl;
    istart = &array_under[0];
    for ( unsigned int i = 0; i < 3; ++i ) {
        *(istart-i) = i;
    }

//    std::cout << "Overrunning array_over." << std::endl;
    istart = &array_over[0];
    for ( unsigned int i = 0; i < array_over.size() + 1; ++i ) {
        *(istart+i) = i;
    }
//
//    std::cout << "Buffer results: " << std::endl;
    EXPECT_TRUE(array_under.bufferUnderRun());
    EXPECT_FALSE(array_under.bufferOverRun());

    EXPECT_TRUE(array_over.bufferOverRun());
    EXPECT_FALSE(array_over.bufferUnderRun());

    EXPECT_FALSE(array_ok.bufferOverRun());
    EXPECT_FALSE(array_ok.bufferUnderRun());

//    std::cout << "Dynamic Arrays" << std::endl;
//    std::cout << std::endl;
//
    Array<int> darray_ok(4); darray_ok << 1, 2, 3, 4;
    Array<int> darray_under(4); darray_under << 1, 2, 3, 4;
    Array<int> darray_over(4); darray_over << 1, 2, 3, 4;

//    std::cout << "Array_under/over/ok: " << darray_ok << std::endl;
//
//    std::cout << "Underrunning array_under." << std::endl;
    istart = &darray_under[0];
    for ( unsigned int i = 0; i < darray_under.size()+1; ++i ) {
        *(istart+i-1) = i;
    }
//
//    std::cout << "Overrunning array_over." << std::endl;
    istart = &darray_over[0];
    for ( unsigned int i = 0; i < darray_over.size()+1; ++i ) {
        *(istart+i) = i;
    }

    EXPECT_TRUE(darray_under.bufferUnderRun());
    EXPECT_FALSE(darray_under.bufferOverRun());

    EXPECT_TRUE(darray_over.bufferOverRun());
    EXPECT_FALSE(darray_over.bufferUnderRun());

    EXPECT_FALSE(darray_ok.bufferOverRun());
    EXPECT_FALSE(darray_ok.bufferUnderRun());
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}

