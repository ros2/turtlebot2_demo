/**
 * @file /src/test/tension_function.cpp
 *
 * @brief Unit Test for tension functions.
 *
 * @date May 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <string>
#include <gtest/gtest.h>
#include <ecl/formatters/floats.hpp>
#include <ecl/formatters/strings.hpp>
#include "../../include/ecl/geometry/tension_function.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using std::string;
using ecl::RightAlign;
using ecl::Format;
using ecl::TensionFunction;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(TensionFunction,allEggsInOneBasket) {
	// Haven't got around to running this properly through gtests yet.
	SUCCEED();
}
/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

	Format<string> string_format; string_format.width(8); string_format.align(RightAlign);
    Format<double> format; format.width(8); format.precision(2); format.align(RightAlign);

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                    Constructors" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    TensionFunction function;
    function = TensionFunction::Interpolation(2.0,1.0,2.0,3.0,2.0,3.0);

    std::cout << "f(tau,x) =" << std::endl;
    std::cout << function << std::endl;

    std::cout << string_format("t=0.01");
    std::cout << string_format("t=1");
    std::cout << string_format("t=2");
    std::cout << string_format("t=3");
    std::cout << string_format("t=4");
    std::cout << string_format("t=10") << std::endl;
    for ( int i = 0; i <= 50; ++i ) {
        std::cout << format(function(0.01,2.0 + 0.02*i));
        for ( int j = 1; j < 5; j+=1 ) {
            std::cout << format(function(1.0*j,2.0 + 0.02*i));
        }
        std::cout << format(function(10.0,2.0 + 0.02*i));
        std::cout << std::endl;
    }

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                    Derivatives" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << string_format("t=0.01");
    std::cout << string_format("t=1");
    std::cout << string_format("t=2");
    std::cout << string_format("t=3");
    std::cout << string_format("t=4");
    std::cout << string_format("t=10") << std::endl;
    for ( int i = 0; i <= 50; ++i ) {
        std::cout << format(function.derivative(0.01,2.0 + 0.02*i));
        for ( int j = 1; j < 5; j+=1 ) {
            std::cout << format(function.derivative(1.0*j,2.0 + 0.02*i));
        }
        std::cout << format(function.derivative(10.0,2.0 + 0.02*i));
        std::cout << std::endl;
    }


    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                  2nd Derivatives" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << string_format("t=0.01");
    std::cout << string_format("t=1");
    std::cout << string_format("t=2");
    std::cout << string_format("t=3");
    std::cout << string_format("t=4");
    std::cout << string_format("t=10") << std::endl;
    for ( int i = 0; i <= 50; ++i ) {
        std::cout << format(function.dderivative(0.01,2.0 + 0.02*i));
        for ( int j = 1; j < 5; j+=1 ) {
            std::cout << format(function.dderivative(1.0*j,2.0 + 0.02*i));
        }
        std::cout << format(function.dderivative(10.0,2.0 + 0.02*i));
        std::cout << std::endl;
    }


    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Passed" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}


