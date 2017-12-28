/**
 * @file /src/test/polynomials.cpp
 *
 * @brief Unit Test for polynomial functions.
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
#include "../../include/ecl/geometry/cartesian_point.hpp"
#include "../../include/ecl/geometry/pascals_triangle.hpp"
#include "../../include/ecl/geometry/polynomial.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using std::string;
using ecl::CartesianPoint2d;
using ecl::RightAlign;
using ecl::Format;
using ecl::Division;
using ecl::Intersection;
using ecl::LinearFunction;
using ecl::QuadraticPolynomial;
using ecl::CubicPolynomial;
using ecl::PascalsTriangle;
using ecl::Polynomial;
using ecl::QuinticPolynomial;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(PolynomialTests,pascalsTriangle) {

//    1  1  1  1
//    1  2  3
//    1  3
//    1
    PascalsTriangle<3> pascals_triangle_three;
    PascalsTriangle<3>::const_iterator iter3 = pascals_triangle_three.begin();
    EXPECT_EQ(2,*(iter3+5));

//    1   1   1   1   1   1
//    1   2   3   4   5
//    1   3   6   10
//    1   4   10
//    1   5
//    1
    PascalsTriangle<5> pascals_triangle;
    PascalsTriangle<5>::const_iterator iter5 = pascals_triangle.begin();
    EXPECT_EQ(3,*(iter5+8));

//    1  1  1  1  1
//    1  2  3  4
//    1  3  6
//    1  4
//    1
    PascalsTriangle<4> pascals_triangle_four;
    PascalsTriangle<4>::const_iterator iter4 = pascals_triangle_four.begin();
    EXPECT_EQ(4,*(iter4+8));
}

TEST(PolynomialTests,quadratic) {
    Polynomial<2> p;
    p.coefficients() << 0, 1, 1;
    EXPECT_EQ(0,p.coefficients()[0]);
    EXPECT_EQ(1,p.coefficients()[1]);
    EXPECT_EQ(1,p.coefficients()[2]);
//    std::cout << "p(5.3) = " << p(5.3) << std::endl;
    // Allow some roundoff error here
    EXPECT_GT(33.40,p(5.3)); EXPECT_LT(33.38,p(5.3));
    p.shift_horizontal(2);
//    std::cout << "p(x-2)(5.3) = " << p(5.3) << std::endl;
    EXPECT_GT(14.20,p(5.3)); EXPECT_LT(14.18,p(5.3));
}

TEST(PolynomialTests,cubic) {
    CubicPolynomial p;
    p.coefficients() << 3, 1, 1, 1;
//    double maximum = ecl::Maximum<CubicPolynomial>()(0.0, 0.2, p);
    double maximum = ecl::CubicPolynomial::Maximum(0.0, 0.2, p);
    //std::cout << p << std::endl;
    //std::cout << "Maximum [0.0, 0.2]: " << maximum << std::endl;
    EXPECT_DOUBLE_EQ(maximum, 3.248);
}

TEST(PolynomialTests,linear_function) {
	LinearFunction f, g;
	f = LinearFunction::PointSlopeForm(1.0,3.0,2.0);
	g = LinearFunction::PointSlopeForm(0.0,4.0,-1.0);
	CartesianPoint2d point = LinearFunction::Intersection(f,g);
    EXPECT_DOUBLE_EQ(point.x(),1.0);
    EXPECT_DOUBLE_EQ(point.y(),3.0);
	//std::cout << "Intersection: " << point << std::endl;
    bool caught = false;
    Intersection<LinearFunction> intersection;
    try {
    	point = intersection(f,f);
    } catch ( ecl::StandardException &e ) {
    	// collinear
    	caught = true;
    }
    EXPECT_EQ(caught, true);
    EXPECT_EQ(intersection.fail(),true);
}

TEST(PolynomialTests,roots) {
	ecl::Array<double> roots;
	LinearFunction f = LinearFunction::PointSlopeForm(1.0,3.0,2.0);
	roots = LinearFunction::Roots(f);
	EXPECT_EQ(1,roots.size());
	if ( roots.size() > 0 ) {
		EXPECT_DOUBLE_EQ(-0.5,roots[0]);
	}
	QuadraticPolynomial q;
	q.coefficients() << 1.0, 2.0, 1.0; // x^2 + 2x + 1 (single root)
	roots = QuadraticPolynomial::Roots(q);
	EXPECT_EQ(1,roots.size());
	if ( roots.size() > 0 ) {
		EXPECT_DOUBLE_EQ(-1.0,roots[0]);
	}
	q.coefficients() << 2.0, 2.0, 1.0; // x^2 + 2x + 2 (no roots)
	roots = QuadraticPolynomial::Roots(q);
	EXPECT_EQ(0,roots.size());
	q.coefficients() << -2.0, 1.0, 1.0; // x^2 + x -2 (two roots)
	roots = QuadraticPolynomial::Roots(q);
	EXPECT_EQ(2,roots.size());
	EXPECT_DOUBLE_EQ(1,roots[0]);
	EXPECT_DOUBLE_EQ(-2,roots[1]);
	CubicPolynomial c;
	c.coefficients() << -6.0, 1.0, 4.0, 1.0; // x^3 + 4x^2 + x - 6 = (x-1)(x+2)(x+3) three roots
	roots = CubicPolynomial::Roots(c);
	EXPECT_EQ(3,roots.size());
	if ( roots.size() == 3 ) {
		EXPECT_DOUBLE_EQ(1,roots[0]);
		EXPECT_DOUBLE_EQ(-2,roots[1]);
		EXPECT_DOUBLE_EQ(-3,roots[2]);
	}
	c.coefficients() << 0, 0, 0, 1.0; // x^3 one triple root
	roots = CubicPolynomial::Roots(c);
	EXPECT_EQ(1,roots.size());
	if ( roots.size() == 1 ) {
		EXPECT_DOUBLE_EQ(0,roots[0]);
	}
	c.coefficients() << -1.0, 1.0, -1.0, 1.0; // (x-1)^3  shifted by one (one real, two complex)
	roots = CubicPolynomial::Roots(c);
	EXPECT_EQ(1,roots.size());
	if ( roots.size() == 1 ) {
		EXPECT_DOUBLE_EQ(1,roots[0]);
	}
}
TEST(PolynomialTests,division) {
	CubicPolynomial c;
	c.coefficients() << -6.0, 1.0, 4.0, 1.0; // x^3 + 4x^2 + x - 6 = (x-1)(x+2)(x+3) three roots
	ecl::FunctionMath<CubicPolynomial> math;
	double remainder;
	QuadraticPolynomial q = CubicPolynomial::Division(c, 1, remainder); // x^2 + 5x + 6
	EXPECT_DOUBLE_EQ(6.0, q.coefficients()[0]);
	EXPECT_DOUBLE_EQ(5.0, q.coefficients()[1]);
	EXPECT_DOUBLE_EQ(1.0, q.coefficients()[2]);
	EXPECT_DOUBLE_EQ(0.0, remainder);
	LinearFunction f = QuadraticPolynomial::Division(q, -2, remainder);
	EXPECT_DOUBLE_EQ(3.0, f.coefficients()[0]); // x + 3
	EXPECT_DOUBLE_EQ(1.0, f.coefficients()[1]);
	EXPECT_DOUBLE_EQ(0.0, remainder);
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

	// Haven't got around to gtesting these yet.

	Format<string> string_format; string_format.width(8); string_format.align(RightAlign);
	Format<double> format; format.width(8); format.precision(2); format.align(RightAlign);

	std::cout << std::endl;
	std::cout << "***********************************************************" << std::endl;
	std::cout << "                  Polynomial Blueprints" << std::endl;
	std::cout << "***********************************************************" << std::endl;
	std::cout << std::endl;

	LinearFunction linear_function = ecl::blueprints::LinearInterpolation(0.0,1.0,1.0,2.0);
	linear_function = LinearFunction::Interpolation(0.0,1.0,1.0,2.0);
	std::cout << "Linear function [interpolation]: " << linear_function << std::endl;
	linear_function = ecl::blueprints::LinearPointSlopeForm(1.0,3.0,2.0);
	linear_function = LinearFunction::PointSlopeForm(1.0,3.0,2.0);
	std::cout << "Linear function   [point-slope]: " << linear_function << std::endl;
	CubicPolynomial cubic;
	cubic = ecl::blueprints::CubicDerivativeInterpolation(2.0,0.0,0.0,3.0,1.0,0.0);
	cubic = CubicPolynomial::SecondDerivativeInterpolation(1.0,0.0,0.0,2.0,1.0,1.0); // same thing
	std::cout << cubic << std::endl;
	cubic = CubicPolynomial::DerivativeInterpolation(2.0,0.0,0.0,3.0,1.0,0.0);
	std::cout << cubic << std::endl;
	QuinticPolynomial quintic;
	quintic = QuinticPolynomial::Interpolation(2.0,0.0,0.0,0.0,
												 3.0,1.0,0.0,0.0);
	std::cout << quintic << std::endl;
	std::cout << std::endl;

	std::cout << "Results" << std::endl;
	std::cout << string_format("cubic");
	std::cout << string_format("quintic") << std::endl;
	for ( int i = 0; i <= 20; ++i ) {
		std::cout << format(cubic(2.0 + 0.05*i));
		std::cout << format(quintic(0.0 + 0.05*i)) << std::endl;
//        std::cout << format(quintic(0.0 + 0.05*i)) << std::endl;
	}

	std::cout << std::endl;
	std::cout << "***********************************************************" << std::endl;
	std::cout << "                  Polynomial Derivatives" << std::endl;
	std::cout << "***********************************************************" << std::endl;
	std::cout << std::endl;

	Polynomial<2> cubic_derivative = cubic.derivative();
	Polynomial<1> cubic_dderivative = cubic_derivative.derivative();

	std::cout << "1st Derivative: " << cubic_derivative << std::endl;
	std::cout << "2nd Derivative: " << cubic_dderivative << std::endl;

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}



