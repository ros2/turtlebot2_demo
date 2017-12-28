/**
 * @file /src/test/cubic_splines.cpp
 *
 * @brief Unit Test for cubic splines.
 *
 * @date 20/05/2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <gtest/gtest.h>
#include <ecl/containers/array.hpp>
#include "../../include/ecl/geometry/cubic_spline.hpp"

//#include <iostream>
//#include <ecl/formatters/floats.hpp>
//#include <ecl/formatters/strings.hpp>

/*****************************************************************************
** Using
*****************************************************************************/

//using std::cout; using std::endl;
using std::string;
using ecl::Array;
//using ecl::RightAlign;
//using ecl::Format;
using ecl::CubicPolynomial;
using ecl::CubicSpline;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(CubicSplinesTests,derivativeHeuristic) {
    Array<double> x_set(3);
    Array<double> y_set(3);
    x_set << 0.688792, 1.15454, 1.67894;
    y_set << -0.75, -1.2, -1.30;
    double ydot_0 = -0.5/x_set[0];
    double ydot_f = 1.04/3.72;

    CubicSpline cubic = CubicSpline::DerivativeHeuristic(x_set, y_set, ydot_0, ydot_f);
    // cout << cubic << endl;
    //
    // -2.08 + 5.96x^1 + -7.85x^2 + 2.90x^3 <---- first cubic
    // -0.22 + -0.72x^1 + -0.46x^2 + 0.30x^3 <---- second cubic
    //
    const Array<double>& domain = cubic.domain();
    EXPECT_EQ(0.688792, domain[0]);
    EXPECT_EQ(1.15454, domain[1]);
    EXPECT_EQ(1.67894, domain[2]);
    const CubicPolynomial &p1 = cubic.polynomials()[0];
    const CubicPolynomial &p2 = cubic.polynomials()[1];
    // allow for some roundoff errors
    EXPECT_GT(-2.07,p1.coefficients()[0]); EXPECT_LT(-2.09,p1.coefficients()[0]);
    EXPECT_GT(5.97,p1.coefficients()[1]); EXPECT_LT(5.95,p1.coefficients()[1]);
    EXPECT_GT(-7.84,p1.coefficients()[2]); EXPECT_LT(-7.86,p1.coefficients()[2]);
    EXPECT_GT(2.91,p1.coefficients()[3]); EXPECT_LT(2.89,p1.coefficients()[3]);

    EXPECT_GT(-0.21,p2.coefficients()[0]); EXPECT_LT(-0.23,p2.coefficients()[0]);
    EXPECT_GT(-0.71,p2.coefficients()[1]); EXPECT_LT(-0.73,p2.coefficients()[1]);
    EXPECT_GT(-0.45,p2.coefficients()[2]); EXPECT_LT(-0.47,p2.coefficients()[2]);
    EXPECT_GT(0.31,p2.coefficients()[3]); EXPECT_LT(0.29,p2.coefficients()[3]);

//    /*********************
//    ** Output
//    **********************/
//    Format<string> string_format; string_format.width(6); string_format.align(RightAlign);
//    Format<double> format; format.width(6); format.precision(2); format.align(RightAlign);
//    cout << string_format("x  ");
//    cout << string_format("y  ");
//    cout << string_format("y' ");
//    cout << string_format("y''") << endl;
//    int n = 50;
//    for ( int i = 0; i <= n; ++i ) {
//        double x = x_set[0] + i*(x_set.back()-x_set.front())/n;
//        cout << format(x);
//        cout << format(cubic(x));
//        cout << format(cubic.derivative(x));
//        cout << format(cubic.dderivative(x)) << endl;
//    }
}


TEST(CubicSplinesTests,continuousHeuristic) {
    Array<double> x_set(3);
    Array<double> y_set(3);
    x_set << 0.688792, 1.15454, 1.67894;
    y_set << -0.75, -1.2, -1.30;
    double ydot_0 = -0.5/x_set[0];
    double ydot_f = 1.04/3.72;
	CubicSpline cubic = CubicSpline::ContinuousDerivatives(x_set, y_set, ydot_0, ydot_f);
    const CubicPolynomial &p1 = cubic.polynomials()[0];
    const CubicPolynomial &p2 = cubic.polynomials()[1];
    // allow for some roundoff errors
    EXPECT_GT(-1.57,p1.coefficients()[0]); EXPECT_LT(-1.59,p1.coefficients()[0]);
    EXPECT_GT(4.10,p1.coefficients()[1]); EXPECT_LT(4.08,p1.coefficients()[1]);
    EXPECT_GT(-5.53,p1.coefficients()[2]); EXPECT_LT(-5.55,p1.coefficients()[2]);
    EXPECT_GT(2.00,p1.coefficients()[3]); EXPECT_LT(1.98,p1.coefficients()[3]);

    EXPECT_GT(2.13,p2.coefficients()[0]); EXPECT_LT(2.11,p2.coefficients()[0]);
    EXPECT_GT(-5.51,p2.coefficients()[1]); EXPECT_LT(-5.53,p2.coefficients()[1]);
    EXPECT_GT(2.79,p2.coefficients()[2]); EXPECT_LT(2.77,p2.coefficients()[2]);
    EXPECT_GT(-0.41,p2.coefficients()[3]); EXPECT_LT(-0.43,p2.coefficients()[3]);
//    /*********************
//    ** Output
//    **********************/
//	  std::cout << cubic << std::endl;
//    Format<string> string_format; string_format.width(6); string_format.align(RightAlign);
//    Format<double> format; format.width(6); format.precision(2); format.align(RightAlign);
//    cout << string_format("x  ");
//    cout << string_format("y  ");
//    cout << string_format("y' ");
//    cout << string_format("y''") << endl;
//    int n = 50;
//    for ( int i = 0; i <= n; ++i ) {
//        double x = x_set[0] + i*(x_set.back()-x_set.front())/n;
//        cout << format(x);
//        cout << format(cubic_c2(x));
//        cout << format(cubic_c2.derivative(x));
//        cout << format(cubic_c2.dderivative(x)) << endl;
//    }
}

TEST(CubicSplinesTests,naturalSpline) {
    Array<double> x_set(3);
    Array<double> y_set(3);
    x_set << 0.688792, 1.15454, 1.67894;
    y_set << -0.75, -1.2, -1.30;

    CubicSpline cubic = CubicSpline::Natural(x_set, y_set);
    const CubicPolynomial &p1 = cubic.polynomials()[0];
    const CubicPolynomial &p2 = cubic.polynomials()[1];
    // allow for some roundoff errors
    EXPECT_GT(-0.22,p1.coefficients()[0]); EXPECT_LT(-0.24,p1.coefficients()[0]);
    EXPECT_GT(0.06,p1.coefficients()[1]); EXPECT_LT(0.04,p1.coefficients()[1]);
    EXPECT_GT(-1.73,p1.coefficients()[2]); EXPECT_LT(-1.75,p1.coefficients()[2]);
    EXPECT_GT(0.85,p1.coefficients()[3]); EXPECT_LT(0.83,p1.coefficients()[3]);

    EXPECT_GT(2.22,p2.coefficients()[0]); EXPECT_LT(2.20,p2.coefficients()[0]);
    EXPECT_GT(-6.29,p2.coefficients()[1]); EXPECT_LT(-6.31,p2.coefficients()[1]);
    EXPECT_GT(3.77,p2.coefficients()[2]); EXPECT_LT(3.75,p2.coefficients()[2]);
    EXPECT_GT(-0.74,p2.coefficients()[3]); EXPECT_LT(-0.76,p2.coefficients()[3]);

//    /*********************
//    ** Output
//    **********************/
//    Format<string> string_format; string_format.width(6); string_format.align(RightAlign);
//    Format<double> format; format.width(6); format.precision(2); format.align(RightAlign);
//    cout << string_format("x  ");
//    cout << string_format("y  ");
//    cout << string_format("y' ");
//    cout << string_format("y''") << endl;
//    int n = 50;
//    for ( int i = 0; i <= n; ++i ) {
//        double x = x_set[0] + i*(x_set.back()-x_set.front())/n;
//        cout << format(x);
//        cout << format(cubic(x));
//        cout << format(cubic.derivative(x));
//        cout << format(cubic.dderivative(x)) << endl;
//    }
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}




