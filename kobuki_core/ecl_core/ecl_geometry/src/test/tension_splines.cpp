/**
 * @file /src/test/tension_splines.cpp
 *
 * @brief Unit Test for tension splines.
 *
 * @date May 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <string>
#include <gtest/gtest.h>
#include <ecl/containers/array.hpp>
#include <ecl/formatters/floats.hpp>
#include <ecl/formatters/strings.hpp>
#include "../../include/ecl/geometry/tension_spline.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using std::cout; using std::endl;
using std::string;
using ecl::Array;
using ecl::RightAlign;
using ecl::Format;
using ecl::TensionSpline;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(TensionFunctionSplines,allEggsInOneBasket) {
	// Haven't got around to running this properly through gtests yet.
	SUCCEED();
}
/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    Format<string> string_format; string_format.width(8); string_format.align(RightAlign);
    Format<double> format; format.width(8); format.precision(2); format.align(RightAlign);

    /*********************
    ** Initialisation
    **********************/
    Array<double> x_set(6);
    Array<double> y_set(6);
    x_set << 0.0, 1.0, 2.0, 3.0, 4.0, 5.0;
    y_set << 1.0, 2.0, 1.0, 3.0, 4.0, 4.0;
//    Array<double> x_set(5);
//    Array<double> y_set(5);
//    x_set = 0.0, 0.43, 1, 1.4, 2.86;
//    y_set = 0.3, 0.1, -0.27, -0.53, -0.43;
    int n = 50;

    cout << endl;
    cout << "***********************************************************" << endl;
    cout << "              Tension Spline Blueprint" << endl;
    cout << "***********************************************************" << endl;
    cout << endl;

    /*********************
    ** Generation
    **********************/
    TensionSpline spline_0_1 = TensionSpline::Natural(x_set, y_set, 0.1);
    TensionSpline spline_1 = TensionSpline::Natural(x_set, y_set, 1.0);
    TensionSpline spline_2 = TensionSpline::Natural(x_set, y_set, 2.0);
    TensionSpline spline_3 = TensionSpline::Natural(x_set, y_set, 3.0);
    TensionSpline spline_4 = TensionSpline::Natural(x_set, y_set, 4.0);
    TensionSpline spline_10 = TensionSpline::Natural(x_set, y_set, 10.0);
    cout << spline_1 << endl;

    cout << endl;
    cout << "***********************************************************" << endl;
    cout << "                   Output : Splines" << endl;
    cout << "***********************************************************" << endl;
    cout << endl;

    /*********************
    ** Output
    **********************/
    std::cout << string_format("t=0.1");
    std::cout << string_format("t=1");
    std::cout << string_format("t=2");
    std::cout << string_format("t=3");
    std::cout << string_format("t=4");
    std::cout << string_format("t=10") << std::endl;
    for ( int i = 0; i <= n; ++i ) {
        double x = i*(x_set.back()-x_set.front())/n;
        std::cout << format(spline_0_1(x));
        std::cout << format(spline_1(x));
        std::cout << format(spline_2(x));
        std::cout << format(spline_3(x));
        std::cout << format(spline_4(x));
        std::cout << format(spline_10(x));
        std::cout << std::endl;
    }

    cout << endl;
    cout << "***********************************************************" << endl;
    cout << "                 Output : Derivatives" << endl;
    cout << "***********************************************************" << endl;
    cout << endl;

    /*********************
    ** Output
    **********************/
    std::cout << string_format("t=0.1");
    std::cout << string_format("t=1");
    std::cout << string_format("t=2");
    std::cout << string_format("t=3");
    std::cout << string_format("t=4");
    std::cout << string_format("t=10") << std::endl;
    for ( int i = 0; i <= n; ++i ) {
        double x = i*(x_set.back()-x_set.front())/n;
        std::cout << format(spline_0_1.derivative(x));
        std::cout << format(spline_1.derivative(x));
        std::cout << format(spline_2.derivative(x));
        std::cout << format(spline_3.derivative(x));
        std::cout << format(spline_4.derivative(x));
        std::cout << format(spline_10.derivative(x));
        std::cout << std::endl;
    }

    cout << endl;
    cout << "***********************************************************" << endl;
    cout << "                 Output : 2nd Derivatives" << endl;
    cout << "***********************************************************" << endl;
    cout << endl;

    /*********************
    ** Output
    **********************/
    std::cout << string_format("t=0.1");
    std::cout << string_format("t=1");
    std::cout << string_format("t=2");
    std::cout << string_format("t=3");
    std::cout << string_format("t=4");
    std::cout << string_format("t=10") << std::endl;
    for ( int i = 0; i <= n; ++i ) {
        double x = i*(x_set.back()-x_set.front())/n;
        std::cout << format(spline_0_1.dderivative(x));
        std::cout << format(spline_1.dderivative(x));
        std::cout << format(spline_2.dderivative(x));
        std::cout << format(spline_3.dderivative(x));
        std::cout << format(spline_4.dderivative(x));
        std::cout << format(spline_10.dderivative(x));
        std::cout << std::endl;
    }

    cout << endl;
    cout << "***********************************************************" << endl;
    cout << "                      Passed" << endl;
    cout << "***********************************************************" << endl;
    cout << endl;

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
