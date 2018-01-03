/**
 * @file /src/test/smooth_linear_splines.cpp
 *
 * @brief Unit Test for smooth linear splines.
 *
 * @date July 2009
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
#include "../../include/ecl/geometry/smooth_linear_spline.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using std::cout; using std::endl;
using std::string;
using ecl::Array;
using ecl::RightAlign;
using ecl::Format;
using ecl::SmoothLinearSpline;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(SmoothLinearSplinesTests,allEggsInOneBasket) {
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
    const double a_max = 10.0;

    cout << endl;
    cout << "***********************************************************" << endl;
    cout << "           Smooth Linear Spline Constructor" << endl;
    cout << "***********************************************************" << endl;
    cout << endl;

    SmoothLinearSpline spline = SmoothLinearSpline::Interpolation(x_set, y_set, a_max);
//    cout << spline_1 << endl;

    cout << endl;
    cout << "***********************************************************" << endl;
    cout << "                      Output" << endl;
    cout << "***********************************************************" << endl;
    cout << endl;

    int n = 200;
    cout << string_format("x  ");
    cout << string_format("y  ");
    cout << string_format("y' ");
    cout << string_format("y''") << endl;
    for ( int i = 0; i <= n; ++i ) {
        double x = x_set[0] + i*(x_set.back()-x_set.front())/n;
        cout << format(x);
        cout << format(spline(x));
        cout << format(spline.derivative(x));
        cout << format(spline.dderivative(x)) << endl;
    }

    cout << endl;
    cout << "***********************************************************" << endl;
    cout << "                      Passed" << endl;
    cout << "***********************************************************" << endl;
    cout << endl;

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}

